/*
 * The INSLIB nav_suite filter on core 1 (plans/inslib-nav-suite.md,
 * plans/rtos-decision.md).
 *
 * Core 0 pushes sensor samples into three queues. Core 1 runs a plain loop
 * with nothing that preempts it: one nav_suite_update() per IMU sample, with
 * any GNSS epoch and barometer sample that arrived attached to it. Every
 * 100 ms it publishes its outputs for core 0 to read.
 *
 * All functions here are called from core 0.
 */
#ifndef NAV_CORE1_H
#define NAV_CORE1_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    NAV_ANT_SINGLE = 0,    /* yaw from the GNSS course over ground          */
    NAV_ANT_DUAL           /* UM982: yaw from the dual-antenna heading      */
} nav_antenna_t;

/* IMU sample in FRD SI units. */
typedef struct {
    uint64_t t_us;         /* sample time, Pico clock                       */
    float    dt_s;         /* spacing to the previous sample                */
    float    acc[3];       /* specific force, m/s^2                         */
    float    gyr[3];       /* rad/s                                         */
    float    mag[3];       /* uT                                            */
    bool     mag_new;
} nav_imu_t;

/* GNSS epoch: a BESTNAV position/velocity, or a dual-antenna heading. Each
 * kind comes as its own item with its own time of validity. */
typedef struct {
    uint64_t t_valid_us;   /* time of validity, Pico clock                  */
    bool     pos_valid;
    bool     vel_valid;
    bool     yaw_valid;
    float    yaw_rad;      /* body yaw, NED, -pi..pi                        */
    float    yaw_sd_rad;
    double   lat_rad;
    double   lon_rad;
    double   h_ell_m;      /* height above the WGS84 ellipsoid              */
    float    pos_sd_ned[3];
    float    vel_ned[3];
    float    vel_sd_ned[3];
    uint16_t week;
    uint32_t tow_ms;
} nav_gnss_t;

typedef struct {
    uint64_t t_us;
    float    pressure_pa;
} nav_baro_t;

/* Published by core 1 every 100 ms. */
typedef struct {
    uint64_t t_us;         /* when it was published, 0 = filter not started */
    int      mode;         /* nav_suite_mode_t                              */
    bool     rpy_valid;
    float    rpy_rad[3];   /* best source: INS, else AHRS, else ARS         */
    bool     pos_valid;
    double   llh[3];       /* lat [rad], lon [rad], ellipsoid height [m]    */
    bool     vel_valid;
    float    vel_ned[3];
    bool     height_valid;
    float    height_m;     /* above the start point, positive up            */
    bool     bias_valid;
    float    gyr_bias[3];  /* rad/s                                         */
    float    acc_bias[3];  /* m/s^2                                         */
    bool     zaru_active;

    uint32_t n_updates;    /* nav_suite_update() calls                      */
    uint32_t n_gnss;       /* GNSS epochs given to the filter               */
    uint32_t n_gnss_late;  /* GNSS epochs dropped: older than INSLIB's limit */
    uint32_t n_yaw;        /* dual-antenna headings given to the filter     */
    bool     dual;         /* antenna mode the filter was started with      */
    uint32_t n_baro;
    uint32_t n_baro_late;  /* barometer samples dropped: too old            */
    uint32_t update_us_max;  /* longest nav_suite_update() since the last report */
    uint32_t update_us_mean; /* mean over the same interval                 */
    uint32_t n_reinit;     /* filter (re)initialisations                    */
} nav_output_t;

/* Filter settings from the web config page (settings.h). */
typedef struct {
    float leverarm_m[3];         /* GNSS antenna from the IMU, body FRD */
    float acc_psd;               /* (m/s^2)^2/Hz */
    float gyr_psd;               /* (rad/s)^2/Hz */
    float gyr_bias_rw;           /* rad/s/sqrt(s) */
    float acc_bias_rw;           /* m/s^2/sqrt(s) */
    float automotive_min_speed;  /* m/s, 0 = INSLIB default */
} nav_params_t;

/* Creates the queues and starts core 1. The filter starts at once with the
 * antenna-mode setting, which sets automotive_mode. Core 1 also registers
 * for flash lockout, so settings_save() can pause it. */
void nav_start(nav_antenna_t mode, const nav_params_t *params);

/* New filter settings: core 1 picks them up before its next sample and
 * re-initialises the filter (re-align, then navigate). Output pauses until
 * then; n_reinit counts it. */
void nav_set_params(const nav_params_t *params);

/* Each returns false if the queue was full and the sample was dropped. */
bool nav_push_imu(const nav_imu_t *s);
bool nav_push_gnss(const nav_gnss_t *s);
bool nav_push_baro(const nav_baro_t *s);

/* The effective antenna mode, once the receiver is identified. Core 1
 * re-initialises the filter only if it differs from the mode it runs with
 * (dual set, but a UM980 fitted). */
void nav_set_antenna_mode(nav_antenna_t mode);

void nav_get_output(nav_output_t *out);

/* Samples dropped because a queue to core 1 was full, per queue. Core 1
 * takes GNSS and baro samples only alongside an IMU sample, so GNSS and
 * baro drops that climb while imu stays flat mean no IMU data is flowing. */
typedef struct {
    uint32_t imu;
    uint32_t gnss;
    uint32_t baro;
} nav_drops_t;

/* Core 0 only (the counters are written by the nav_push_*() calls). */
void nav_queue_drops(nav_drops_t *out);

/* Next INSLIB log message, if any. Print it from core 0. */
bool nav_pop_log(char *buf, size_t len);

/* Highest core 1 stack use so far [bytes], and the stack size. */
uint32_t nav_stack_used(void);
uint32_t nav_stack_size(void);

const char *nav_mode_name(int mode);

#endif /* NAV_CORE1_H */
