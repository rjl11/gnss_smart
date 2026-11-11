/*
 * Settings of the demo that are not pins: mounting, noise figures, options.
 * Pins and baud rates are in board_pins.h.
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* TM210 accelerometer sign (plans/imu-tm210.md, "Decide on the bench").
 * 1: the TM210 reports the gravity vector, FRD = (-ax, ay, az) * g.
 * 0: the TM210 reports specific force,     FRD = (ax, -ay, -az) * g.
 * Check: level and at rest, the status line must show acc z near -9.81.
 * If it shows +9.81, flip this value. */
#define APP_TM210_ACC_IS_GRAVITY    1

/* GNSS antenna position relative to the IMU, body frame FRD [m].
 * Measure on the vehicle. Zero is right for a bench test with the antenna
 * close to the IMU. */
#define APP_GNSS_LEVERARM_X_M       0.0f
#define APP_GNSS_LEVERARM_Y_M       0.0f
#define APP_GNSS_LEVERARM_Z_M       0.0f

/* IMU noise, start values from the datasheet (plans/imu-tm210.md,
 * "INSLIB noise parameters"). Replace with Allan variance results. */
#define APP_ACC_PSD                 3.5e-6f   /* (m/s^2)^2/Hz */
#define APP_GYR_PSD                 1.5e-8f   /* (rad/s)^2/Hz */
#define APP_GYR_BIAS_RW             5.0e-6f   /* rad/s/sqrt(s) */
#define APP_ACC_BIAS_RW             3.0e-4f   /* m/s^2/sqrt(s) */

/* 0 = INSLIB default (2 m/s). Below this speed the GNSS course is not used
 * for yaw (plans/inslib-nav-suite.md). */
#define APP_AUTOMOTIVE_MIN_SPEED    0.0f

/* The magnetometer is not fed to INSLIB until it is calibrated on the
 * vehicle. Without it the AHRS filter stays off and yaw comes from the INS
 * (GNSS course) or the free-running ARS. */
#define APP_FEED_MAG                0

/* Core 1 waits this long for the receiver model before it starts the filter
 * with the UM980 setting (start value, not yet measured). */
#define APP_MODEL_WAIT_MS           10000

/* Core 1 stack. nav_suite_update() needs about 13 KB worst case. */
#define APP_CORE1_STACK_BYTES       (32 * 1024)

#endif /* APP_CONFIG_H */
