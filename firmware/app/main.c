/*
 * gnss_smart demo: sensor bring-up and INSLIB fused output.
 *
 * Core 0 (this file): UART DMA, the TM210 and UM980 drivers with their boot
 * provisioning, the BMP581, and a status report on USB once a second.
 * Core 1 (nav_core1.c): the INSLIB nav_suite filter.
 *
 * No Bluetooth in this build. Pins are in board_pins.h, other settings in
 * app_config.h.
 */
#include <math.h>
#include <stdio.h>

#include "pico/stdlib.h"

#include "app_config.h"
#include "board_pins.h"
#include "bmp581.h"
#include "nav_core1.h"
#include "tm210.h"
#include "uart_dma.h"
#include "um980.h"

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
    printf("[bmp581] running\n");
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

static void print_status(uint64_t now)
{
    const tm210_stats_t *is = tm210_stats(&s_imu);
    const um980_stats_t *gs = um980_stats(&s_gnss);
    const bmp581_stats_t *bs = bmp581_stats(&s_baro);
    static uint32_t s_baro_prev;
    nav_output_t o;

    nav_get_output(&o);

    printf("\n--- t=%.1f s ---\n", (double)now * 1e-6);

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
    if (um980_model(&s_gnss) != 0) {
        printf("  %s build %lu", um980_model(&s_gnss) == UM980_PRODUCT_UM980 ? "UM980" :
               um980_model(&s_gnss) == UM980_PRODUCT_UM982 ? "UM982" : "other",
               (unsigned long)s_gnss.build);
    }
    if (um980_last_error(&s_gnss) != UM980_OK) {
        printf("  [%s]", um980_strerror(um980_last_error(&s_gnss)));
    }
    printf("\n");
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
               (unsigned long)(bs->samples - s_baro_prev), (unsigned long)bs->i2c_errors,
               (double)s_last_baro.pressure_pa, (double)s_last_baro.temperature_c);
        s_baro_prev = bs->samples;
    } else {
        printf("BARO  not found\n");
    }

    if (o.t_us == 0) {
        printf("NAV   waiting for the receiver model\n");
    } else {
        printf("NAV   %-15s update mean %lu us, max %lu us  stack %lu/%lu B  drops %lu\n",
               nav_mode_name(o.mode), (unsigned long)o.update_us_mean,
               (unsigned long)o.update_us_max, (unsigned long)nav_stack_used(),
               (unsigned long)nav_stack_size(), (unsigned long)nav_queue_drops());
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
        printf("      epochs: imu %lu  gnss %lu (late %lu)  baro %lu\n", (unsigned long)o.n_updates,
               (unsigned long)o.n_gnss, (unsigned long)o.n_gnss_late, (unsigned long)o.n_baro);
    }
}

int main(void)
{
    stdio_init_all();

    /* Give a terminal a moment to attach, so the provisioning log is seen. */
    absolute_time_t usb_until = make_timeout_time_ms(USB_WAIT_MS);
    while (!stdio_usb_connected() && !time_reached(usb_until)) {
        sleep_ms(10);
    }
    printf("\ngnss_smart demo: TM210 + UM980 + BMP581 -> INSLIB nav_suite\n");

    if (!uart_dma_init(&s_imu_port, PIN_TM210_UART, PIN_TM210_TX, PIN_TM210_RX, TM210_BAUD,
                       s_imu_rx, IMU_RX_BITS) ||
        !uart_dma_init(&s_gnss_port, PIN_UM980_UART, PIN_UM980_TX, PIN_UM980_RX, UM980_BAUD,
                       s_gnss_rx, GNSS_RX_BITS)) {
        panic("no free DMA channel");
    }

    const tm210_config_t imu_cfg = { .acc_is_gravity = APP_TM210_ACC_IS_GRAVITY, .verbose = true };
    const um980_config_t gnss_cfg = { .pps_pin = PIN_UM980_PPS, .verbose = true };
    tm210_init(&s_imu, &s_imu_port, &imu_cfg);
    um980_init(&s_gnss, &s_gnss_port, &gnss_cfg);
    s_baro_ok = baro_start();

    nav_start();

    uint64_t next_status = time_us_64() + STATUS_PERIOD_US;
    uint64_t next_baro_try = time_us_64() + BARO_RETRY_US;
    int model_sent = 0;

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
        while (um980_poll(&s_gnss, &gn)) {
            forward_gnss(&gn);
            s_last_nav = gn;
            s_have_nav = true;
        }
        if (um980_model(&s_gnss) != model_sent) {
            model_sent = um980_model(&s_gnss);
            nav_set_receiver_model(model_sent == UM980_PRODUCT_UM982 ? NAV_RX_DUAL_ANTENNA
                                                                      : NAV_RX_SINGLE_ANTENNA);
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

        if (now >= next_status) {
            print_status(now);
            next_status += STATUS_PERIOD_US;
        }
    }
}
