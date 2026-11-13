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

/* The TM210 magnetometer is fed to INSLIB (the AHRS and the INS). INSLIB
 * applies its own World Magnetic Model for the declination and the expected
 * field strength once core 1 has a GNSS position, and gates samples whose
 * field strength is off (a steel tractor). 0 leaves the AHRS off and yaw
 * comes from the INS (GNSS course or dual-antenna heading) or the ARS. */
#define APP_FEED_MAG                1

/* Estimate the magnetometer hard-iron bias in the INS (18 error states).
 * It converges while the vehicle turns. The on-vehicle calibration
 * (inslib_mag_calib.py, plans/imu-tm210.md) will add fixed soft- and
 * hard-iron terms later. */
#define APP_ESTIMATE_MAG_BIAS       1

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
 * Wi-Fi runs"). Set at build time with cmake -DWEB_MODE=always|switch|off.
 * switch: GPIO22 (PIN_WEB_MODE_SWITCH, internal pull-up) is read once at
 * boot; pulled to GND = Wi-Fi and the web server on, open = off for this
 * boot. The station SSID and password come from cmake too (WIFI_STA_SSID,
 * WIFI_STA_PASSWORD; an empty password means an open network). */
#define APP_WEB_OFF                 0
#define APP_WEB_ALWAYS              1
#define APP_WEB_SWITCH              2
#ifndef APP_WEB_MODE
#define APP_WEB_MODE                APP_WEB_OFF
#endif
/* The web code is linked in (always or switch); whether it runs this boot is
 * decided at runtime in switch mode. */
#define APP_WEB_BUILT               (APP_WEB_MODE != APP_WEB_OFF)
#ifndef APP_WIFI_STA_SSID
#define APP_WIFI_STA_SSID           "TeliaGateway"
#endif
#ifndef APP_WIFI_STA_PASSWORD
#define APP_WIFI_STA_PASSWORD       ""
#endif
/* Network mode (cmake -DWIFI_MODE=auto|sta|ap). auto: join the station
 * network; if that hasn't worked within APP_WIFI_AP_FALLBACK_S of boot, run
 * the open AP instead until the next reboot. sta: station only. ap: open AP
 * from boot. The AP is open (no password) by decision, at 192.168.4.1. */
#define APP_WIFI_AUTO               0
#define APP_WIFI_STA                1
#define APP_WIFI_AP                 2
#ifndef APP_WIFI_MODE
#define APP_WIFI_MODE               APP_WIFI_AUTO
#endif
#define APP_WIFI_AP_SSID            "gnss_smart"
#define APP_WIFI_AP_FALLBACK_S      30

/* DHCP and mDNS name: http://gnss-smart.local/ where mDNS resolves. */
#define APP_WEB_HOSTNAME            "gnss-smart"

/* Core 1 stack. nav_suite_update() needs about 13 KB worst case. */
#define APP_CORE1_STACK_BYTES       (32 * 1024)

#endif /* APP_CONFIG_H */
