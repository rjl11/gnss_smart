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
    NAV_RX_UNKNOWN = 0,
    NAV_RX_SINGLE_ANTENNA, /* UM980: yaw from the GNSS course over ground   */
    NAV_RX_DUAL_ANTENNA    /* UM982: yaw from the receiver's heading        */
} nav_receiver_t;

/* IMU sample in FRD SI units. */
typedef struct {
    uint64_t t_us;         /* sample time, Pico clock                       */
    float    dt_s;         /* spacing to the previous sample                */
    float    acc[3];       /* specific force, m/s^2                         */
    float    gyr[3];       /* rad/s                                         */
    float    mag[3];       /* uT                                            */
    bool     mag_new;
} nav_imu_t;

/* GNSS epoch. */
typedef struct {
    uint64_t t_valid_us;   /* time of validity, Pico clock                  */
    bool     pos_valid;
    bool     vel_valid;
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
    uint32_t n_baro;
    uint32_t update_us_max;  /* longest nav_suite_update() since the last report */
    uint32_t update_us_mean; /* mean over the same interval                 */
    uint32_t n_reinit;     /* filter (re)initialisations                    */
} nav_output_t;

/* Creates the queues and starts core 1. */
void nav_start(void);

/* Each returns false if the queue was full and the sample was dropped. */
bool nav_push_imu(const nav_imu_t *s);
bool nav_push_gnss(const nav_gnss_t *s);
bool nav_push_baro(const nav_baro_t *s);

/* Core 1 holds nav_suite_init() until this is known (or APP_MODEL_WAIT_MS has
 * passed), and initialises again if the model changes later. */
void nav_set_receiver_model(nav_receiver_t model);

void nav_get_output(nav_output_t *out);

/* Samples dropped because a queue to core 1 was full. */
uint32_t nav_queue_drops(void);

/* Next INSLIB log message, if any. Print it from core 0. */
bool nav_pop_log(char *buf, size_t len);

/* Highest core 1 stack use so far [bytes], and the stack size. */
uint32_t nav_stack_used(void);
uint32_t nav_stack_size(void);

const char *nav_mode_name(int mode);

#endif /* NAV_CORE1_H */
