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

/* GNSS antenna mode (plans/gnss-um98x-config.md, "Antenna mode").
 * Single: UM980, or UM982 with the slave antenna off; yaw from the GNSS
 * course over ground. Dual: UM982 with both antennas; yaw from its heading.
 * Dual with a UM980 fitted falls back to single. Set it at build time with
 * cmake -DGNSS_ANTENNA_MODE=dual. The web page will override it from flash
 * once it exists; until then this default is the setting. */
#define APP_ANTENNA_SINGLE          0
#define APP_ANTENNA_DUAL            1
#ifndef APP_GNSS_ANTENNA_MODE
#define APP_GNSS_ANTENNA_MODE       APP_ANTENNA_SINGLE
#endif

/* Dual antenna only. Direction of the ANT1 -> ANT2 baseline in the body
 * frame, clockwise from the body x-axis (forward) [deg]. Across the cab,
 * left to right, is +90. Measure on the vehicle. */
#define APP_BASELINE_YAW_DEG        90.0f

/* Dual antenna only. A heading epoch is dropped when its baseline length
 * differs from the running median by more than this (start value, tune on
 * the bench). */
#define APP_BASELINE_GATE_M         0.05f

/* Dual antenna only. Floor on the heading sigma, for a 1 m baseline; it
 * scales with 1 / length (datasheet: 0.1 deg RMS at 1 m). */
#define APP_HEADING_SD_FLOOR_DEG_1M 0.1f

/* A barometer sample older than this when it reaches the filter is dropped
 * (start value, not yet measured). */
#define APP_BARO_MAX_AGE_US         100000

/* When Wi-Fi and the web server run (plans/web-config-server.md, "When
 * Wi-Fi runs"). Set at build time with cmake -DWEB_MODE=always|off; switch
 * (GPIO22 at boot) is Phase 5. The station SSID and password come from cmake
 * too (WIFI_STA_SSID, WIFI_STA_PASSWORD; an empty password means an open
 * network). */
#define APP_WEB_OFF                 0
#define APP_WEB_ALWAYS              1
#ifndef APP_WEB_MODE
#define APP_WEB_MODE                APP_WEB_OFF
#endif
#ifndef APP_WIFI_STA_SSID
#define APP_WIFI_STA_SSID           "TeliaGateway"
#endif
#ifndef APP_WIFI_STA_PASSWORD
#define APP_WIFI_STA_PASSWORD       ""
#endif
/* DHCP and mDNS name: http://gnss-smart.local/ where mDNS resolves. */
#define APP_WEB_HOSTNAME            "gnss-smart"

/* Core 1 stack. nav_suite_update() needs about 13 KB worst case. */
#define APP_CORE1_STACK_BYTES       (32 * 1024)

#endif /* APP_CONFIG_H */
