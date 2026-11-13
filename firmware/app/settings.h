/*
 * User settings: defaults from app_config.h / cmake, overridden by the copy
 * saved in flash from the web config page (plans/web-config-server.md,
 * "Configuration page" and "Storing configuration").
 *
 * Flash layout: the last two 4 KB sectors, written alternately. Each holds
 * one record (header with a sequence number and a CRC, then the struct); the
 * valid record with the higher sequence number wins, so a power cut during a
 * save leaves the previous one.
 *
 * Core 0 only.
 */
#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SETTINGS_SSID_MAX   32
#define SETTINGS_PW_MAX     63

/* Bump SETTINGS_VERSION whenever this struct changes: a record with another
 * version is ignored and the defaults are used. */
#define SETTINGS_VERSION    1

typedef struct {
    /* Filter: applied live, the filter re-initialises. */
    float leverarm_m[3];         /* GNSS antenna from the IMU, body FRD */
    float acc_psd;               /* (m/s^2)^2/Hz */
    float gyr_psd;               /* (rad/s)^2/Hz */
    float gyr_bias_rw;           /* rad/s/sqrt(s) */
    float acc_bias_rw;           /* m/s^2/sqrt(s) */
    float automotive_min_speed;  /* m/s, 0 = INSLIB default */

    /* Dual-antenna heading: applied live on core 0. */
    float baseline_yaw_deg;
    float baseline_gate_m;

    /* Applied after a reboot. */
    uint8_t antenna_dual;        /* 0 single, 1 dual */
    uint8_t wifi_mode;           /* APP_WIFI_AUTO / _STA / _AP */
    char sta_ssid[SETTINGS_SSID_MAX + 1];
    char sta_password[SETTINGS_PW_MAX + 1]; /* "" = open network */
} settings_t;

void settings_defaults(settings_t *s);

/* Loads the newest valid record, or the defaults if there is none. Returns
 * true if the values came from flash. */
bool settings_load(settings_t *s);

/* NULL if every field is in range, else a message naming the field. */
const char *settings_validate(const settings_t *s);

/* Writes a new record into the older sector. Pauses core 1 (flash lockout)
 * and core 0's interrupts for one sector erase, about 50 ms (estimate, not
 * yet measured). Returns 0 or a negative error. */
int settings_save(const settings_t *s);

/* Erases both sectors: the next boot uses the defaults. */
int settings_erase(void);

/* Sequence number of the record in use, 0 = defaults. */
uint32_t settings_seq(void);

/* True if the fields that only apply at boot differ between a and b. */
bool settings_reboot_fields_differ(const settings_t *a, const settings_t *b);

#endif /* SETTINGS_H */
