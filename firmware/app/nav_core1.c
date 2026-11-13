#include "nav_core1.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "pico/critical_section.h"
#include "pico/flash.h"
#include "pico/multicore.h"
#include "pico/time.h"
#include "pico/util/queue.h"

#include "log.h"
#include "nav_suite.h"

#include "app_config.h"

#define IMU_QUEUE_LEN       128 /* 160 ms at 800 Hz */
#define GNSS_QUEUE_LEN      8
#define BARO_QUEUE_LEN      32
#define LOG_QUEUE_LEN       8
#define LOG_MSG_LEN         120
#define PUBLISH_PERIOD_US   100000u
#define STACK_FILL          0xA5A5A5A5u

static queue_t s_imu_q, s_gnss_q, s_baro_q, s_log_q;
static critical_section_t s_out_lock;
static nav_output_t s_out;                 /* guarded by s_out_lock */
static volatile nav_antenna_t s_mode;
static nav_drops_t s_drops;                /* core 0 only */
static nav_params_t s_params_new;          /* guarded by s_out_lock */
static volatile uint32_t s_params_seq;     /* bumped by core 0 on a change */

/* AAPCS wants the stack pointer on an 8-byte boundary. */
static uint32_t s_stack[APP_CORE1_STACK_BYTES / sizeof(uint32_t)] __attribute__((aligned(8)));

/* Core 1 only. */
static nav_params_t s_p;                   /* settings the filter runs with */
static nav_suite_t s_suite;
static nav_output_t s_work;
static uint64_t s_update_us_sum;
static uint32_t s_update_count;
static bool s_mag_model_set;

/* ---- Core 1 ------------------------------------------------------------ */

/* INSLIB's messages are formatted here and printed by core 0, so core 1
 * never waits for USB. */
static void log_sink(int level, const char *file, int line, const char *fmt, va_list args)
{
    static const char *const k_level[] = { "", "FATAL", "ERROR", "WARN", "INFO" };
    char msg[LOG_MSG_LEN];
    /* File name only: the build's absolute path would fill the message. */
    for (const char *p = file; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\') {
            file = p + 1;
        }
    }
    int n = snprintf(msg, sizeof msg, "%s %s:%d: ", k_level[(level >= 1 && level <= 4) ? level : 0],
                     file, line);
    if (n > 0 && (size_t)n < sizeof msg) {
        vsnprintf(msg + n, sizeof msg - (size_t)n, fmt, args);
    }
    queue_try_add(&s_log_q, msg);
}

static void filter_init(nav_antenna_t mode)
{
    ins_init_t init;
    ins_options_t opt;

    memset(&s_suite, 0, sizeof s_suite);
    memset(&init, 0, sizeof init);
    memset(&opt, 0, sizeof opt);

    /* Position stays zero: auto-init replaces it with the first good fix. */
    init.time = (ins_time_us_t)time_us_64();
    init.gyr_bias_pred_stddev_rps_sqrts = s_p.gyr_bias_rw;
    init.acc_bias_pred_stddev_mps2_sqrts = s_p.acc_bias_rw;

    opt.auto_init = true;
    /* Course yaw only with one antenna. In dual mode it adds nothing and
     * pulls the wrong way when reversing (plans/gnss-um98x-config.md). */
    opt.automotive_mode = mode != NAV_ANT_DUAL;
    opt.automotive_min_speed_mps = s_p.automotive_min_speed;
    opt.estimate_mag_bias = APP_FEED_MAG && APP_ESTIMATE_MAG_BIAS;

    nav_suite_init(&s_suite, &init, &opt);
    s_mag_model_set = false;
    s_work.dual = mode == NAV_ANT_DUAL;
    s_work.n_reinit++;
}

static void set_diag(float *dst, float a, float b, float c)
{
    /* 3x3 column-major, diagonal only */
    dst[0] = a;
    dst[4] = b;
    dst[8] = c;
}

static void attach_gnss(ins_measurements_t *m, const nav_gnss_t *g)
{
    int64_t age_us = (int64_t)m->timestamp - (int64_t)g->t_valid_us;
    if (age_us < 0) {
        age_us = 0;
    }
    if (age_us >= (int64_t)INS_MAX_DELAY_MS * 1000) {
        s_work.n_gnss_late++;
        return;
    }
    if (g->yaw_valid) {
        m->yaw.yaw_rad = g->yaw_rad;
        m->yaw.stddev_rad = g->yaw_sd_rad;
        m->yaw.is_valid = true;
        m->yaw_delay_ms = (int)(age_us / 1000);
        m->yaw_delay_us = (int32_t)(age_us % 1000);
        s_work.n_yaw++;
    }
    if (!g->pos_valid && !g->vel_valid) {
        return;
    }

    if (g->pos_valid) {
        m->gnss_pos.llh[0] = g->lat_rad;
        m->gnss_pos.llh[1] = g->lon_rad;
        m->gnss_pos.llh[2] = g->h_ell_m;
        set_diag(m->gnss_pos.Qll_ned, g->pos_sd_ned[0] * g->pos_sd_ned[0],
                 g->pos_sd_ned[1] * g->pos_sd_ned[1], g->pos_sd_ned[2] * g->pos_sd_ned[2]);
        m->gnss_pos.is_valid = true;
    }
    if (g->vel_valid) {
        memcpy(m->gnss_vel.vel_ned, g->vel_ned, sizeof m->gnss_vel.vel_ned);
        set_diag(m->gnss_vel.Qll_ned, g->vel_sd_ned[0] * g->vel_sd_ned[0],
                 g->vel_sd_ned[1] * g->vel_sd_ned[1], g->vel_sd_ned[2] * g->vel_sd_ned[2]);
        m->gnss_vel.is_valid = true;
    }
    m->gnss_leverarm_b[0] = s_p.leverarm_m[0];
    m->gnss_leverarm_b[1] = s_p.leverarm_m[1];
    m->gnss_leverarm_b[2] = s_p.leverarm_m[2];
    m->gps_week = g->week;
    m->gps_itow_ms = (int)g->tow_ms;
    m->gnss_delay_ms = (int)(age_us / 1000);
    m->gnss_delay_us = (int32_t)(age_us % 1000);
    s_work.n_gnss++;

    if (g->pos_valid && !s_mag_model_set) {
        /* GPS week 0 began on 1980-01-06. */
        float year = 1980.0f + (5.0f + (float)g->week * 7.0f) / 365.25f;
        nav_suite_set_magnetic_model_position(&s_suite, g->lat_rad, g->lon_rad, year);
        s_mag_model_set = true;
    }
}

static void publish(uint64_t now)
{
    nav_output_t *o = &s_work;

    o->t_us = now;
    o->mode = (int)nav_suite_get_mode(&s_suite);
    o->rpy_valid = nav_suite_get_rpy(&s_suite, &o->rpy_rad[0], &o->rpy_rad[1], &o->rpy_rad[2]);
    o->pos_valid = ins_is_ready(&s_suite.ins) && ins_get_latlonh(&s_suite.ins, o->llh);
    o->vel_valid = ins_is_ready(&s_suite.ins) && ins_get_velocity_ned(&s_suite.ins, o->vel_ned);
    o->height_valid = nav_suite_get_height(&s_suite, &o->height_m);
    o->bias_valid = ins_get_bias_gyr(&s_suite.ins, o->gyr_bias) &&
                    ins_get_bias_acc(&s_suite.ins, o->acc_bias);
    o->zaru_active = nav_suite_get_zaru_active(&s_suite);
    o->update_us_mean = s_update_count ? (uint32_t)(s_update_us_sum / s_update_count) : 0;

    critical_section_enter_blocking(&s_out_lock);
    s_out = *o;
    critical_section_exit(&s_out_lock);

    o->update_us_max = 0;
    s_update_us_sum = 0;
    s_update_count = 0;
}

static void core1_main(void)
{
    nav_imu_t imu;
    nav_gnss_t gnss;
    nav_baro_t baro;
    ins_measurements_t m;

    log_set_sink(log_sink);
    /* Lets core 0 pause this core while it writes the settings to flash. */
    flash_safe_execute_core_init();

    /* The antenna mode is a setting known at boot, so the filter starts at
     * once. It restarts if the receiver can't do that mode, or when the
     * settings change. */
    uint32_t params_seq = s_params_seq;
    critical_section_enter_blocking(&s_out_lock);
    s_p = s_params_new;
    critical_section_exit(&s_out_lock);
    nav_antenna_t mode = s_mode;
    filter_init(mode);
    uint64_t next_publish = time_us_64() + PUBLISH_PERIOD_US;

    for (;;) {
        nav_antenna_t new_mode = s_mode;
        uint32_t new_seq = s_params_seq;
        if (new_mode != mode || new_seq != params_seq) {
            if (new_seq != params_seq) {
                params_seq = new_seq;
                critical_section_enter_blocking(&s_out_lock);
                s_p = s_params_new;
                critical_section_exit(&s_out_lock);
            }
            mode = new_mode;
            filter_init(mode);
        }

        if (queue_try_remove(&s_imu_q, &imu)) {
            memset(&m, 0, sizeof m);
            m.timestamp = (ins_time_us_t)imu.t_us;
            m.strapdown_dt_sec = imu.dt_s;
            memcpy(m.acc.data, imu.acc, sizeof m.acc.data);
            memcpy(m.gyr.data, imu.gyr, sizeof m.gyr.data);
            for (int i = 0; i < 3; i++) {
                m.acc.Qll_diag[i] = s_p.acc_psd;
                m.gyr.Qll_diag[i] = s_p.gyr_psd;
            }
            m.acc.is_valid = true;
            m.gyr.is_valid = true;
#if APP_FEED_MAG
            if (imu.mag_new) {
                memcpy(m.mag.data, imu.mag, sizeof m.mag.data);
                m.mag.is_valid = true;
            }
#endif
            /* Samples that arrived since the last epoch ride on this one.
             * is_valid is set only for a new sample. */
            if (queue_try_remove(&s_gnss_q, &gnss)) {
                attach_gnss(&m, &gnss);
            }
            if (queue_try_remove(&s_baro_q, &baro)) {
                /* The barometer has no delay compensation in INSLIB, so a
                 * sample that waited in the queue is not fused as current. */
                if ((int64_t)imu.t_us - (int64_t)baro.t_us > APP_BARO_MAX_AGE_US) {
                    s_work.n_baro_late++;
                } else {
                    m.baro.pressure_pa = baro.pressure_pa;
                    m.baro.is_valid = true;
                    s_work.n_baro++;
                }
            }

            uint64_t t0 = time_us_64();
            nav_suite_update(&s_suite, &m);
            uint32_t took = (uint32_t)(time_us_64() - t0);

            s_work.n_updates++;
            s_update_us_sum += took;
            s_update_count++;
            if (took > s_work.update_us_max) {
                s_work.update_us_max = took;
            }
        }

        uint64_t now = time_us_64();
        if (now >= next_publish) {
            publish(now);
            next_publish += PUBLISH_PERIOD_US;
            if (next_publish < now) {
                next_publish = now + PUBLISH_PERIOD_US;
            }
        }
    }
}

/* ---- Core 0 ------------------------------------------------------------ */

void nav_set_params(const nav_params_t *params)
{
    critical_section_enter_blocking(&s_out_lock);
    s_params_new = *params;
    critical_section_exit(&s_out_lock);
    s_params_seq++;
}

void nav_start(nav_antenna_t mode, const nav_params_t *params)
{
    s_mode = mode;
    s_params_new = *params;
    queue_init(&s_imu_q, sizeof(nav_imu_t), IMU_QUEUE_LEN);
    queue_init(&s_gnss_q, sizeof(nav_gnss_t), GNSS_QUEUE_LEN);
    queue_init(&s_baro_q, sizeof(nav_baro_t), BARO_QUEUE_LEN);
    queue_init(&s_log_q, LOG_MSG_LEN, LOG_QUEUE_LEN);
    critical_section_init(&s_out_lock);

    for (size_t i = 0; i < sizeof s_stack / sizeof s_stack[0]; i++) {
        s_stack[i] = STACK_FILL;
    }
    multicore_launch_core1_with_stack(core1_main, s_stack, sizeof s_stack);
}

static bool push(queue_t *q, const void *item, uint32_t *drops)
{
    if (queue_try_add(q, item)) {
        return true;
    }
    (*drops)++;
    return false;
}

/* A dropped sample's dt is added to the next one, so the filter integrates
 * over the whole interval and not only over the last sample period. */
bool nav_push_imu(const nav_imu_t *s)
{
    static float s_lost_dt_s; /* core 0 only */
    nav_imu_t n = *s;

    n.dt_s += s_lost_dt_s;
    if (push(&s_imu_q, &n, &s_drops.imu)) {
        s_lost_dt_s = 0.0f;
        return true;
    }
    s_lost_dt_s = n.dt_s;
    return false;
}

bool nav_push_gnss(const nav_gnss_t *s) { return push(&s_gnss_q, s, &s_drops.gnss); }
bool nav_push_baro(const nav_baro_t *s) { return push(&s_baro_q, s, &s_drops.baro); }

void nav_set_antenna_mode(nav_antenna_t mode) { s_mode = mode; }

void nav_get_output(nav_output_t *out)
{
    critical_section_enter_blocking(&s_out_lock);
    *out = s_out;
    critical_section_exit(&s_out_lock);
}

void nav_queue_drops(nav_drops_t *out) { *out = s_drops; }

bool nav_pop_log(char *buf, size_t len)
{
    char msg[LOG_MSG_LEN];
    if (!queue_try_remove(&s_log_q, msg)) {
        return false;
    }
    msg[LOG_MSG_LEN - 1] = '\0';
    snprintf(buf, len, "%s", msg);
    return true;
}

uint32_t nav_stack_used(void)
{
    /* The stack grows down from the end of the array. */
    size_t n = sizeof s_stack / sizeof s_stack[0];
    size_t untouched = 0;
    while (untouched < n && s_stack[untouched] == STACK_FILL) {
        untouched++;
    }
    return (uint32_t)((n - untouched) * sizeof(uint32_t));
}

uint32_t nav_stack_size(void) { return (uint32_t)sizeof s_stack; }

const char *nav_mode_name(int mode)
{
    switch (mode) {
    case NAV_SUITE_MODE_NONE:          return "NONE";
    case NAV_SUITE_MODE_ATTITUDE_ONLY: return "ATTITUDE_ONLY";
    case NAV_SUITE_MODE_COASTING:      return "COASTING";
    case NAV_SUITE_MODE_FULL:          return "FULL";
    default:                           return "?";
    }
}
