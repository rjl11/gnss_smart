/*
 * gnss_smart demo: sensor bring-up and INSLIB fused output.
 *
 * Core 0 (this file): UART DMA, the TM210 and UM980 drivers with their boot
 * provisioning, the BMP581, and a status report on USB once a second.
 * Core 1 (nav_core1.c): the INSLIB nav_suite filter.
 *
 * Wi-Fi and the web server (app/web/) run when built with WEB_MODE=always:
 * station or open AP, status page with /status.json, config page with
 * /config.json and POST /config (plans/web-config-server.md, Phases 2-4).
 * Settings come from flash or the build defaults (settings.h).
 *
 * No Bluetooth in this build. Pins are in board_pins.h, other settings in
 * app_config.h.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "app_config.h"
#include "board_pins.h"
#include "bmp581.h"
#include "nav_core1.h"
#include "settings.h"
#include "tm210.h"
#include "uart_dma.h"
#include "um980.h"
#if APP_WEB_BUILT
#include "hardware/watchdog.h"
#include "web/config_web.h"
#include "web/jsonw.h"
#include "web/web.h"
#endif

#define IMU_RX_BITS         13 /* 8 KB, about 200 ms at 39 kB/s */
#define GNSS_RX_BITS        12 /* 4 KB */
#define USB_WAIT_MS         3000
#define STATUS_PERIOD_US    1000000u
#define BARO_RETRY_US       5000000u
#define DEG                 (180.0 / M_PI)

static uint8_t s_imu_rx[1u << IMU_RX_BITS] __attribute__((aligned(1u << IMU_RX_BITS)));
static uint8_t s_gnss_rx[1u << GNSS_RX_BITS] __attribute__((aligned(1u << GNSS_RX_BITS)));

static uart_dma_t s_imu_port, s_gnss_port;
static tm210_t s_imu;
static um980_t s_gnss;
static bmp581_t s_baro;
static bool s_baro_ok;

static tm210_sample_t s_last_imu;
static um980_nav_t s_last_nav;
static bmp581_sample_t s_last_baro;
static bool s_have_nav;

/* Dual-antenna heading: baseline-length gate (plans/gnss-um98x-config.md,
 * "VARIABLELENGTH and the mount"). The median is over the last fixed epochs. */
#define BASELINE_HIST       31
#define BASELINE_MIN_HIST   5
static float s_base_hist[BASELINE_HIST];
static unsigned s_base_n, s_base_pos;
static um980_heading_t s_last_hdg;
static bool s_have_hdg;
static float s_base_median_m;
static uint32_t s_hdg_used, s_hdg_gated;

/* Settings in use (settings.h): loaded at boot, live fields replaced by a
 * save from the web page. s_boot_settings keeps what this boot started with,
 * for the fields that only apply after a reboot. */
static settings_t s_settings, s_boot_settings;
static bool s_settings_from_flash;

#if APP_WEB_BUILT
/* Wi-Fi and the web server run this boot: always in WEB_MODE=always, in
 * WEB_MODE=switch only when GPIO22 was pulled to GND at boot. */
static bool s_web_on;
#endif

static void nav_params_from_settings(nav_params_t *p, const settings_t *s)
{
    for (int i = 0; i < 3; i++) {
        p->leverarm_m[i] = s->leverarm_m[i];
    }
    p->acc_psd = s->acc_psd;
    p->gyr_psd = s->gyr_psd;
    p->gyr_bias_rw = s->gyr_bias_rw;
    p->acc_bias_rw = s->acc_bias_rw;
    p->automotive_min_speed = s->automotive_min_speed;
}

static bool baro_start(void)
{
    bmp581_config_t cfg;
    bmp581_default_config(&cfg);
    cfg.i2c = PIN_BMP581_I2C;
    cfg.sda_pin = PIN_BMP581_SDA;
    cfg.scl_pin = PIN_BMP581_SCL;
    cfg.baud_hz = BMP581_I2C_HZ;
    cfg.int_pin = PIN_BMP581_INT;

    int rc = bmp581_init(&s_baro, &cfg);
    if (rc != BMP581_OK) {
        printf("[bmp581] init failed: %s (%d), retrying\n", bmp581_strerror(rc), rc);
        return false;
    }
    printf("[bmp581] running at 0x%02x\n", s_baro.cfg.addr);
    return true;
}

static void forward_gnss(const um980_nav_t *n)
{
    nav_gnss_t g = { 0 };
    double track = n->track_deg / DEG;

    g.t_valid_us = n->t_valid_us;
    g.pos_valid = n->pos_usable;
    g.vel_valid = n->vel_usable;
    g.lat_rad = n->lat_deg / DEG;
    g.lon_rad = n->lon_deg / DEG;
    g.h_ell_m = n->hgt_msl_m + (double)n->undulation_m; /* INSLIB wants ellipsoid height */
    g.pos_sd_ned[0] = n->sd_lat_m;
    g.pos_sd_ned[1] = n->sd_lon_m;
    g.pos_sd_ned[2] = n->sd_hgt_m;
    g.vel_ned[0] = (float)(n->hor_speed_mps * cos(track));
    g.vel_ned[1] = (float)(n->hor_speed_mps * sin(track));
    g.vel_ned[2] = (float)-n->vert_speed_mps;
    g.vel_sd_ned[0] = n->sd_hor_speed_mps;
    g.vel_sd_ned[1] = n->sd_hor_speed_mps;
    g.vel_sd_ned[2] = n->sd_vert_speed_mps;
    g.week = n->week;
    g.tow_ms = n->tow_ms;

    if (g.pos_valid || g.vel_valid) {
        nav_push_gnss(&g);
    }
}

static float median(const float *v, unsigned n)
{
    float s[BASELINE_HIST];
    for (unsigned i = 0; i < n; i++) {
        s[i] = v[i];
    }
    for (unsigned i = 1; i < n; i++) { /* insertion sort, n <= 31 at 20 Hz */
        float x = s[i];
        unsigned j = i;
        while (j > 0 && s[j - 1] > x) {
            s[j] = s[j - 1];
            j--;
        }
        s[j] = x;
    }
    return s[n / 2];
}

/* A fixed heading whose baseline length agrees with the running median goes
 * to core 1 as the body yaw. The first few fixed epochs only fill the
 * median. */
static void forward_heading(const um980_heading_t *h)
{
    if (!h->fixed) {
        return;
    }
    bool gate_ok = false;
    if (s_base_n >= BASELINE_MIN_HIST) {
        s_base_median_m = median(s_base_hist, s_base_n);
        gate_ok = fabsf(h->length_m - s_base_median_m) <= s_settings.baseline_gate_m;
    }
    s_base_hist[s_base_pos] = h->length_m;
    s_base_pos = (s_base_pos + 1u) % BASELINE_HIST;
    if (s_base_n < BASELINE_HIST) {
        s_base_n++;
    }
    if (!gate_ok) {
        if (s_base_n > BASELINE_MIN_HIST) {
            s_hdg_gated++;
        }
        return;
    }

    /* Heading is clockwise from true north, as NED yaw. */
    double yaw = (double)(h->heading_deg - s_settings.baseline_yaw_deg) / DEG;
    yaw = remainder(yaw, 2.0 * M_PI);
    float sd = h->sd_heading_deg;
    float floor_deg = APP_HEADING_SD_FLOOR_DEG_1M / h->length_m;
    if (sd < floor_deg) {
        sd = floor_deg;
    }

    nav_gnss_t g = { 0 };
    g.t_valid_us = h->t_valid_us;
    g.yaw_valid = true;
    g.yaw_rad = (float)yaw;
    g.yaw_sd_rad = (float)(sd / DEG);
    g.week = h->week;
    g.tow_ms = h->tow_ms;
    if (nav_push_gnss(&g)) {
        s_hdg_used++;
    }
}

static const char *fix_name(uint32_t pos_type)
{
    switch (pos_type) {
    case UM980_TYPE_NONE:           return "none";
    case UM980_TYPE_SINGLE:         return "single";
    case UM980_TYPE_PSRDIFF:        return "dgps";
    case UM980_TYPE_SBAS:           return "sbas";
    case UM980_TYPE_L1_FLOAT:
    case UM980_TYPE_IONOFREE_FLOAT:
    case UM980_TYPE_NARROW_FLOAT:   return "rtk-float";
    case UM980_TYPE_L1_INT:
    case UM980_TYPE_WIDE_INT:
    case UM980_TYPE_NARROW_INT:     return "rtk-fixed";
    default:                        return pos_type >= UM980_TYPE_PPP_CONVERGING ? "ppp" : "other";
    }
}

static const char *model_name(void)
{
    switch (um980_model(&s_gnss)) {
    case 0:                    return NULL;
    case UM980_PRODUCT_UM980:  return "UM980";
    case UM980_PRODUCT_UM982:  return "UM982";
    default:                   return "other";
    }
}

static void print_status(uint64_t now, uint32_t baro_hz)
{
    const tm210_stats_t *is = tm210_stats(&s_imu);
    const um980_stats_t *gs = um980_stats(&s_gnss);
    const bmp581_stats_t *bs = bmp581_stats(&s_baro);
    nav_output_t o;

    nav_get_output(&o);

    printf("\n--- t=%.1f s ---\n", (double)now * 1e-6);

#if APP_WEB_BUILT
    web_info_t wi;
    web_get_info(&wi);
    const uint8_t *ip = (const uint8_t *)&wi.ip;
    if (!s_web_on) {
        printf("WIFI  off             (web-mode switch open at boot)\n");
    } else {
    printf("WIFI  %-15s %s", web_state_name(wi.state), web_ssid());
    if (wi.state == WEB_WIFI_UP) {
        printf("  http://%u.%u.%u.%u/  %ld dBm", ip[0], ip[1], ip[2], ip[3], (long)wi.rssi_dbm);
    } else if (wi.state == WEB_WIFI_AP) {
        printf("  http://%u.%u.%u.%u/  clients %lu", ip[0], ip[1], ip[2], ip[3],
               (unsigned long)wi.ap_clients);
    } else if (wi.state == WEB_WIFI_WAIT_RETRY) {
        printf("  retry in %lu s", (unsigned long)wi.retry_in_s);
    }
    printf("  joins %lu  drops %lu\n", (unsigned long)wi.joins, (unsigned long)wi.drops);
    }
#endif

    printf("IMU   %-15s %4lu Hz  crc %lu  gaps %lu  ", tm210_state_name(tm210_state(&s_imu)),
           (unsigned long)is->rate_hz, (unsigned long)is->crc_errors, (unsigned long)is->gaps);
    if (is->status_valid) {
        printf("%.1f C  qos %u", (double)is->temperature_c, is->qos);
    }
    if (tm210_last_error(&s_imu) != TM210_OK) {
        printf("  [%s]", tm210_strerror(tm210_last_error(&s_imu)));
    }
    printf("\n");
    if (tm210_state(&s_imu) == TM210_ST_RUN) {
        printf("      acc %7.3f %7.3f %7.3f m/s2   gyr %7.4f %7.4f %7.4f rad/s\n",
               (double)s_last_imu.acc[0], (double)s_last_imu.acc[1], (double)s_last_imu.acc[2],
               (double)s_last_imu.gyr[0], (double)s_last_imu.gyr[1], (double)s_last_imu.gyr[2]);
    }

    printf("GNSS  %-15s %4lu Hz  crc %lu  pps %lu", um980_state_name(um980_state(&s_gnss)),
           (unsigned long)gs->nav_rate_hz, (unsigned long)gs->crc_errors,
           (unsigned long)gs->pps_edges);
    if (model_name() != NULL) {
        printf("  %s build %lu", model_name(), (unsigned long)s_gnss.build);
        printf("  %s antenna", um980_dual(&s_gnss) ? "dual" : "single");
    }
    if (um980_last_error(&s_gnss) != UM980_OK) {
        printf("  [%s]", um980_strerror(um980_last_error(&s_gnss)));
    }
    printf("\n");
    if (um980_dual_refused(&s_gnss)) {
        printf("      warning: dual antenna is set, but a UM980 is fitted: running single\n");
    }
    if (um980_dual(&s_gnss) && s_have_hdg) {
        printf("      heading %.2f deg  sd %.2f  pitch %.2f  %s  baseline %.3f m (median %.3f)"
               "  used %lu  gated %lu\n",
               (double)s_last_hdg.heading_deg, (double)s_last_hdg.sd_heading_deg,
               (double)s_last_hdg.pitch_deg, fix_name(s_last_hdg.pos_type),
               (double)s_last_hdg.length_m, (double)s_base_median_m, (unsigned long)s_hdg_used,
               (unsigned long)s_hdg_gated);
    }
    if (s_have_nav) {
        printf("      %s  sv %u/%u  hdop %.1f  age %.1f s  time %s\n", fix_name(s_last_nav.pos_type),
               s_last_nav.n_used, s_last_nav.n_tracked, gs->dop_valid ? (double)gs->hdop : 0.0,
               (double)s_last_nav.diff_age_s, s_last_nav.t_from_pps ? "pps" : "arrival");
        if (s_last_nav.pos_usable) {
            printf("      lat %.8f  lon %.8f  h %.3f m  sd %.3f %.3f %.3f m\n", s_last_nav.lat_deg,
                   s_last_nav.lon_deg, s_last_nav.hgt_msl_m + (double)s_last_nav.undulation_m,
                   (double)s_last_nav.sd_lat_m, (double)s_last_nav.sd_lon_m,
                   (double)s_last_nav.sd_hgt_m);
        }
    }

    if (s_baro_ok) {
        printf("BARO  run             %4lu Hz  i2c errors %lu  %.1f Pa  %.1f C\n",
               (unsigned long)baro_hz, (unsigned long)bs->i2c_errors,
               (double)s_last_baro.pressure_pa, (double)s_last_baro.temperature_c);
    } else {
        printf("BARO  not found\n");
    }

    if (o.t_us == 0) {
        printf("NAV   starting\n");
    } else {
        nav_drops_t d;
        nav_queue_drops(&d);
        printf("NAV   %-15s %s  update mean %lu us, max %lu us  stack %lu/%lu B\n",
               nav_mode_name(o.mode), o.dual ? "dual" : "single", (unsigned long)o.update_us_mean,
               (unsigned long)o.update_us_max, (unsigned long)nav_stack_used(),
               (unsigned long)nav_stack_size());
        printf("      queue drops: imu %lu  gnss %lu  baro %lu\n", (unsigned long)d.imu,
               (unsigned long)d.gnss, (unsigned long)d.baro);
        if (o.rpy_valid) {
            printf("      roll %7.2f  pitch %7.2f  yaw %7.2f deg%s\n", (double)o.rpy_rad[0] * DEG,
                   (double)o.rpy_rad[1] * DEG, (double)o.rpy_rad[2] * DEG,
                   o.zaru_active ? "  (standing still)" : "");
        }
        if (o.pos_valid) {
            printf("      lat %.8f  lon %.8f  h %.3f m\n", o.llh[0] * DEG, o.llh[1] * DEG, o.llh[2]);
        }
        if (o.vel_valid) {
            printf("      vel N %.3f  E %.3f  D %.3f m/s\n", (double)o.vel_ned[0],
                   (double)o.vel_ned[1], (double)o.vel_ned[2]);
        }
        if (o.height_valid) {
            printf("      height above start %.2f m\n", (double)o.height_m);
        }
        if (o.bias_valid) {
            printf("      gyro bias %.5f %.5f %.5f rad/s   acc bias %.4f %.4f %.4f m/s2\n",
                   (double)o.gyr_bias[0], (double)o.gyr_bias[1], (double)o.gyr_bias[2],
                   (double)o.acc_bias[0], (double)o.acc_bias[1], (double)o.acc_bias[2]);
        }
        printf("      epochs: imu %lu  gnss %lu (late %lu)  yaw %lu  baro %lu (late %lu)\n",
               (unsigned long)o.n_updates, (unsigned long)o.n_gnss, (unsigned long)o.n_gnss_late,
               (unsigned long)o.n_yaw, (unsigned long)o.n_baro, (unsigned long)o.n_baro_late);
        printf("      rx ring overflows: imu %lu  gnss %lu\n",
               (unsigned long)s_imu_port.rx_overflows, (unsigned long)s_gnss_port.rx_overflows);
    }
}

#if APP_WEB_BUILT
static void jw_vec3(jw_t *w, const char *key, const float *v, int decimals)
{
    jw_arr(w, key);
    for (int i = 0; i < 3; i++) {
        jw_num(w, NULL, (double)v[i], decimals);
    }
    jw_end(w, ']');
}

/* The same data as print_status(), as JSON for /status.json. */
static void publish_status_json(uint64_t now, uint32_t baro_hz)
{
    static char buf[2048];
    const tm210_stats_t *is = tm210_stats(&s_imu);
    const um980_stats_t *gs = um980_stats(&s_gnss);
    const bmp581_stats_t *bs = bmp581_stats(&s_baro);
    nav_output_t o;
    web_info_t wi;
    jw_t w;

    nav_get_output(&o);
    web_get_info(&wi);
    jw_init(&w, buf, sizeof buf);
    jw_obj(&w, NULL);
    jw_str(&w, "fw", "gnss_smart demo");
    jw_int(&w, "uptime_s", (long long)(now / 1000000u));

    char ip[16];
    const uint8_t *b = (const uint8_t *)&wi.ip;
    snprintf(ip, sizeof ip, "%u.%u.%u.%u", b[0], b[1], b[2], b[3]);
    jw_obj(&w, "wifi");
    jw_str(&w, "state", web_state_name(wi.state));
    jw_str(&w, "ssid", web_ssid());
    jw_str(&w, "ip", ip);
    if (wi.state == WEB_WIFI_AP) {
        jw_int(&w, "ap_clients", wi.ap_clients);
    } else if (wi.state == WEB_WIFI_UP) {
        jw_int(&w, "rssi_dbm", wi.rssi_dbm);
    } else {
        jw_null(&w, "rssi_dbm");
    }
    jw_int(&w, "joins", wi.joins);
    jw_int(&w, "drops", wi.drops);
    jw_int(&w, "http_busy", wi.http_busy);
    jw_end(&w, '}');

    jw_obj(&w, "imu");
    jw_str(&w, "state", tm210_state_name(tm210_state(&s_imu)));
    jw_str(&w, "error", tm210_last_error(&s_imu) == TM210_OK ? "ok" :
                        tm210_strerror(tm210_last_error(&s_imu)));
    jw_int(&w, "rate_hz", is->rate_hz);
    jw_int(&w, "crc_errors", is->crc_errors);
    jw_int(&w, "gaps", is->gaps);
    if (is->status_valid) {
        jw_num(&w, "temp_c", (double)is->temperature_c, 1);
        jw_int(&w, "qos", is->qos);
    } else {
        jw_null(&w, "temp_c");
        jw_null(&w, "qos");
    }
    if (tm210_state(&s_imu) == TM210_ST_RUN) {
        jw_vec3(&w, "acc_mps2", s_last_imu.acc, 3);
        jw_vec3(&w, "gyr_rps", s_last_imu.gyr, 4);
    }
    jw_end(&w, '}');

    jw_obj(&w, "gnss");
    jw_str(&w, "state", um980_state_name(um980_state(&s_gnss)));
    jw_str(&w, "error", um980_last_error(&s_gnss) == UM980_OK ? "ok" :
                        um980_strerror(um980_last_error(&s_gnss)));
    jw_int(&w, "rate_hz", gs->nav_rate_hz);
    jw_int(&w, "crc_errors", gs->crc_errors);
    jw_int(&w, "pps_edges", gs->pps_edges);
    if (model_name() != NULL) {
        jw_str(&w, "model", model_name());
        jw_int(&w, "build", (long long)s_gnss.build);
        jw_str(&w, "antenna", um980_dual(&s_gnss) ? "dual" : "single");
    } else {
        jw_null(&w, "model");
    }
    jw_bool(&w, "dual_refused", um980_dual_refused(&s_gnss));
    if (s_have_nav) {
        jw_str(&w, "fix", fix_name(s_last_nav.pos_type));
        jw_int(&w, "sv_used", s_last_nav.n_used);
        jw_int(&w, "sv_tracked", s_last_nav.n_tracked);
        if (gs->dop_valid) {
            jw_num(&w, "hdop", (double)gs->hdop, 1);
        } else {
            jw_null(&w, "hdop");
        }
        jw_num(&w, "age_s", (double)s_last_nav.diff_age_s, 1);
        jw_str(&w, "time_from", s_last_nav.t_from_pps ? "pps" : "arrival");
        if (s_last_nav.pos_usable) {
            jw_num(&w, "lat_deg", s_last_nav.lat_deg, 8);
            jw_num(&w, "lon_deg", s_last_nav.lon_deg, 8);
            jw_num(&w, "h_ell_m", s_last_nav.hgt_msl_m + (double)s_last_nav.undulation_m, 3);
            float sd[3] = { s_last_nav.sd_lat_m, s_last_nav.sd_lon_m, s_last_nav.sd_hgt_m };
            jw_vec3(&w, "sd_m", sd, 3);
        }
    }
    jw_end(&w, '}');

    if (um980_dual(&s_gnss) && s_have_hdg) {
        jw_obj(&w, "heading");
        jw_num(&w, "heading_deg", (double)s_last_hdg.heading_deg, 2);
        jw_num(&w, "sd_deg", (double)s_last_hdg.sd_heading_deg, 2);
        jw_num(&w, "pitch_deg", (double)s_last_hdg.pitch_deg, 2);
        jw_str(&w, "fix", fix_name(s_last_hdg.pos_type));
        jw_num(&w, "baseline_m", (double)s_last_hdg.length_m, 3);
        jw_num(&w, "median_m", (double)s_base_median_m, 3);
        jw_int(&w, "used", s_hdg_used);
        jw_int(&w, "gated", s_hdg_gated);
        jw_end(&w, '}');
    }

    jw_obj(&w, "baro");
    jw_bool(&w, "found", s_baro_ok);
    if (s_baro_ok) {
        jw_int(&w, "rate_hz", baro_hz);
        jw_int(&w, "i2c_errors", bs->i2c_errors);
        jw_num(&w, "pressure_pa", (double)s_last_baro.pressure_pa, 1);
        jw_num(&w, "temp_c", (double)s_last_baro.temperature_c, 1);
    }
    jw_end(&w, '}');

    jw_obj(&w, "nav");
    if (o.t_us == 0) {
        jw_str(&w, "mode", "starting");
    } else {
        jw_str(&w, "mode", nav_mode_name(o.mode));
        jw_str(&w, "antenna", o.dual ? "dual" : "single");
        jw_int(&w, "update_mean_us", o.update_us_mean);
        jw_int(&w, "update_max_us", o.update_us_max);
        jw_int(&w, "stack_used_b", nav_stack_used());
        jw_int(&w, "stack_size_b", nav_stack_size());
        nav_drops_t d;
        nav_queue_drops(&d);
        jw_obj(&w, "queue_drops");
        jw_int(&w, "imu", d.imu);
        jw_int(&w, "gnss", d.gnss);
        jw_int(&w, "baro", d.baro);
        jw_end(&w, '}');
        jw_int(&w, "reinits", o.n_reinit);
        if (o.rpy_valid) {
            float rpy[3] = { (float)(o.rpy_rad[0] * DEG), (float)(o.rpy_rad[1] * DEG),
                             (float)(o.rpy_rad[2] * DEG) };
            jw_vec3(&w, "rpy_deg", rpy, 2);
            jw_bool(&w, "standing_still", o.zaru_active);
        }
        if (o.pos_valid) {
            jw_num(&w, "lat_deg", o.llh[0] * DEG, 8);
            jw_num(&w, "lon_deg", o.llh[1] * DEG, 8);
            jw_num(&w, "h_ell_m", o.llh[2], 3);
        }
        if (o.vel_valid) {
            jw_vec3(&w, "vel_ned_mps", o.vel_ned, 3);
        }
        if (o.height_valid) {
            jw_num(&w, "height_above_start_m", (double)o.height_m, 2);
        }
        if (o.bias_valid) {
            jw_vec3(&w, "gyr_bias_rps", o.gyr_bias, 5);
            jw_vec3(&w, "acc_bias_mps2", o.acc_bias, 4);
        }
        jw_obj(&w, "epochs");
        jw_int(&w, "imu", o.n_updates);
        jw_int(&w, "gnss", o.n_gnss);
        jw_int(&w, "gnss_late", o.n_gnss_late);
        jw_int(&w, "yaw", o.n_yaw);
        jw_int(&w, "baro", o.n_baro);
        jw_int(&w, "baro_late", o.n_baro_late);
        jw_end(&w, '}');
    }
    jw_end(&w, '}');

    jw_obj(&w, "rx_overflows");
    jw_int(&w, "imu", s_imu_port.rx_overflows);
    jw_int(&w, "gnss", s_gnss_port.rx_overflows);
    jw_end(&w, '}');

    jw_end(&w, '}');
    if (!jw_ok(&w)) {
        static const char k_err[] = "{\"error\":\"status JSON too long\"}";
        web_set_status_json(k_err, sizeof k_err - 1);
        return;
    }
    web_set_status_json(buf, w.len);
}
#endif

#if APP_WEB_BUILT
static uint64_t s_reboot_at;

/* Live fields: the filter settings go to core 1, which re-initialises; the
 * heading gate and baseline direction are read from s_settings on core 0. */
static void apply_live(const settings_t *s)
{
    nav_params_t old_p, new_p;
    nav_params_from_settings(&old_p, &s_settings);
    nav_params_from_settings(&new_p, s);
    if (memcmp(&old_p, &new_p, sizeof old_p) != 0) {
        nav_set_params(&new_p);
    }
    s_settings = *s;
}

/* A command from the web config page (config_web.h). */
static void handle_config_cmd(uint64_t now)
{
    config_cmd_t cmd;
    if (!config_web_take(&cmd)) {
        return;
    }
    char res[160];
    switch (cmd.kind) {
    case CFG_CMD_SAVE: {
        uint64_t t0 = time_us_64();
        int rc = settings_save(&cmd.settings);
        unsigned long ms = (unsigned long)((time_us_64() - t0) / 1000u);
        if (rc != 0) {
            snprintf(res, sizeof res, "save failed: flash error %d; nothing changed", rc);
            break;
        }
        apply_live(&cmd.settings);
        s_settings_from_flash = true;
        snprintf(res, sizeof res, "saved as record %lu in %lu ms%s", (unsigned long)settings_seq(), ms,
                 settings_reboot_fields_differ(&s_settings, &s_boot_settings)
                     ? "; reboot to apply the antenna / Wi-Fi changes" : "");
        break;
    }
    case CFG_CMD_DEFAULTS: {
        int rc = settings_erase();
        if (rc != 0) {
            snprintf(res, sizeof res, "erase failed: flash error %d; nothing changed", rc);
            break;
        }
        settings_t d;
        settings_defaults(&d);
        apply_live(&d);
        s_settings_from_flash = false;
        snprintf(res, sizeof res, "build defaults restored%s",
                 settings_reboot_fields_differ(&s_settings, &s_boot_settings)
                     ? "; reboot to apply the antenna / Wi-Fi changes" : "");
        break;
    }
    case CFG_CMD_REBOOT:
        s_reboot_at = now + 500000u; /* lets the reply and the next poll go out */
        snprintf(res, sizeof res, "rebooting");
        break;
    case CFG_CMD_RECONFIGURE_GNSS:
        um980_reconfigure(&s_gnss);
        snprintf(res, sizeof res, "GNSS receiver: full re-provision started");
        break;
    case CFG_CMD_RECONFIGURE_IMU:
        tm210_reconfigure(&s_imu);
        snprintf(res, sizeof res, "IMU: provisioning sequence started");
        break;
    default:
        return;
    }
    printf("[config] %s\n", res);
    config_web_publish(&s_settings, &s_boot_settings, s_settings_from_flash, res);
}
#endif

int main(void)
{
    stdio_init_all();

    /* Give a terminal a moment to attach, so the provisioning log is seen. */
    absolute_time_t usb_until = make_timeout_time_ms(USB_WAIT_MS);
    while (!stdio_usb_connected() && !time_reached(usb_until)) {
        sleep_ms(10);
    }
    printf("\ngnss_smart demo: TM210 + UM980 + BMP581 -> INSLIB nav_suite\n");

    /* Settings from flash, or the build defaults. A record that fails the
     * range check (e.g. written by an older build) is not used. */
    s_settings_from_flash = settings_load(&s_settings);
    const char *bad = settings_validate(&s_settings);
    if (bad != NULL) {
        printf("[settings] saved record rejected (%s), using defaults\n", bad);
        settings_defaults(&s_settings);
        s_settings_from_flash = false;
    }
    s_boot_settings = s_settings;
    printf("[settings] %s\n", s_settings_from_flash ? "loaded from flash" : "build defaults");

#if APP_WEB_BUILT
#if APP_WEB_MODE == APP_WEB_SWITCH
    /* Production: the web-mode switch is read once, here. Pulled to GND =
     * Wi-Fi on for this boot; open (internal pull-up) = off, so the radio,
     * its current draw and its interrupts stay away from the filter. */
    gpio_init(PIN_WEB_MODE_SWITCH);
    gpio_set_dir(PIN_WEB_MODE_SWITCH, GPIO_IN);
    gpio_pull_up(PIN_WEB_MODE_SWITCH);
    sleep_ms(2); /* let the pull-up charge the switch wiring */
    s_web_on = !gpio_get(PIN_WEB_MODE_SWITCH);
    printf("[web] web-mode switch (GP%d) %s: Wi-Fi %s\n", PIN_WEB_MODE_SWITCH,
           s_web_on ? "to GND" : "open", s_web_on ? "on" : "off");
#else
    s_web_on = true;
#endif
    if (s_web_on) {
        /* Before the sensor UARTs start: loading the CYW43439 firmware blocks for
         * a few hundred ms, longer than the IMU ring holds. The join itself runs
         * in the background from web_poll(). */
        const web_config_t web_cfg = {
            .mode = s_settings.wifi_mode == APP_WIFI_AP  ? WEB_NET_AP :
                    s_settings.wifi_mode == APP_WIFI_STA ? WEB_NET_STA : WEB_NET_AUTO,
            .ssid = s_settings.sta_ssid, .password = s_settings.sta_password,
            .ap_ssid = APP_WIFI_AP_SSID, .ap_fallback_s = APP_WIFI_AP_FALLBACK_S,
            .hostname = APP_WEB_HOSTNAME,
        };
        uint64_t t_web = time_us_64();
        bool web_ok = web_init(&web_cfg);
        config_web_init();
        config_web_publish(&s_settings, &s_boot_settings, s_settings_from_flash, NULL);
        printf("[web] init %s in %lu ms\n", web_ok ? "done" : "failed",
               (unsigned long)((time_us_64() - t_web) / 1000u));
    }
#endif

    if (!uart_dma_init(&s_imu_port, PIN_TM210_UART, PIN_TM210_TX, PIN_TM210_RX, TM210_BAUD,
                       s_imu_rx, IMU_RX_BITS) ||
        !uart_dma_init(&s_gnss_port, PIN_UM980_UART, PIN_UM980_TX, PIN_UM980_RX, UM980_BAUD,
                       s_gnss_rx, GNSS_RX_BITS)) {
        panic("no free DMA channel");
    }

    const tm210_config_t imu_cfg = { .acc_is_gravity = APP_TM210_ACC_IS_GRAVITY, .verbose = true };
    /* The antenna-mode setting: build default, or the web page's saved value. */
    const bool dual_set = s_settings.antenna_dual != 0;
    printf("GNSS antenna mode setting: %s\n", dual_set ? "dual" : "single");

    const um980_config_t gnss_cfg = {
        .pps_pin = PIN_UM980_PPS, .dual_antenna = dual_set, .verbose = true,
    };
    tm210_init(&s_imu, &s_imu_port, &imu_cfg);
    um980_init(&s_gnss, &s_gnss_port, &gnss_cfg);
    s_baro_ok = baro_start();

    nav_antenna_t ant_sent = dual_set ? NAV_ANT_DUAL : NAV_ANT_SINGLE;
    nav_params_t np;
    nav_params_from_settings(&np, &s_settings);
    nav_start(ant_sent, &np);

    uint64_t next_status = time_us_64() + STATUS_PERIOD_US;
    uint64_t next_baro_try = time_us_64() + BARO_RETRY_US;

    for (;;) {
        /* IMU: every sample goes to core 1. */
        tm210_sample_t is;
        while (tm210_poll(&s_imu, &is)) {
            nav_imu_t n;
            n.t_us = is.t_us;
            n.dt_s = is.dt_s;
            for (int i = 0; i < 3; i++) {
                n.acc[i] = is.acc[i];
                n.gyr[i] = is.gyr[i];
                n.mag[i] = is.mag[i];
            }
            n.mag_new = is.mag_new;
            nav_push_imu(&n);
            s_last_imu = is;
        }

        /* GNSS: epochs flow once the receiver is configured. */
        um980_nav_t gn;
        um980_heading_t gh;
        for (;;) {
            bool have_nav = um980_poll(&s_gnss, &gn);
            if (um980_take_heading(&s_gnss, &gh)) {
                forward_heading(&gh);
                s_last_hdg = gh;
                s_have_hdg = true;
            }
            if (!have_nav) {
                break;
            }
            forward_gnss(&gn);
            s_last_nav = gn;
            s_have_nav = true;
        }
        /* Once the receiver is identified, the filter follows the effective
         * mode. That differs from the setting only when dual is set and a
         * UM980 is fitted. */
        if (um980_model(&s_gnss) != 0) {
            nav_antenna_t ant = um980_dual(&s_gnss) ? NAV_ANT_DUAL : NAV_ANT_SINGLE;
            if (ant != ant_sent) {
                ant_sent = ant;
                nav_set_antenna_mode(ant);
            }
        }

        /* Barometer. */
        uint64_t now = time_us_64();
        if (s_baro_ok) {
            bmp581_sample_t bs;
            if (bmp581_poll(&s_baro, &bs)) {
                nav_baro_t b = { .t_us = bs.t_us, .pressure_pa = bs.pressure_pa };
                nav_push_baro(&b);
                s_last_baro = bs;
            }
        } else if (now >= next_baro_try) {
            s_baro_ok = baro_start();
            next_baro_try = now + BARO_RETRY_US;
        }

        /* INSLIB messages from core 1. */
        char msg[128];
        while (nav_pop_log(msg, sizeof msg)) {
            printf("[inslib] %s\n", msg);
        }

#if APP_WEB_BUILT
        if (s_web_on) {
            web_poll(now);
            handle_config_cmd(now);
        }
        if (s_reboot_at != 0 && now >= s_reboot_at) {
            printf("[config] reboot\n");
            watchdog_reboot(0, 0, 0);
            for (;;) {
                tight_loop_contents();
            }
        }
#endif

        if (now >= next_status) {
            static uint32_t s_baro_prev;
            uint32_t baro_hz = bmp581_stats(&s_baro)->samples - s_baro_prev;
            s_baro_prev = bmp581_stats(&s_baro)->samples;

            print_status(now, baro_hz);
#if APP_WEB_BUILT
            if (s_web_on) {
                publish_status_json(now, baro_hz);
            }
#endif
            next_status += STATUS_PERIOD_US;
        }
    }
}
