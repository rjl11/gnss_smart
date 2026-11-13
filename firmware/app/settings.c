#include "settings.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/flash.h"
#include "pico/flash.h"

#include "app_config.h"

#define REC_MAGIC       0x46435347u /* "GSCF" */
#define SECTOR_A        (PICO_FLASH_SIZE_BYTES - 2u * FLASH_SECTOR_SIZE)
#define SECTOR_B        (PICO_FLASH_SIZE_BYTES - 1u * FLASH_SECTOR_SIZE)
#define FLASH_TIMEOUT_MS 200u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;      /* sizeof(settings_t) */
    uint32_t seq;
    uint32_t crc;       /* CRC-32 of the settings_t bytes */
    settings_t s;
} record_t;

/* One flash page per record keeps the write a single program call. */
_Static_assert(sizeof(record_t) <= FLASH_PAGE_SIZE, "settings record must fit in one flash page");

static uint32_t s_seq;          /* record in use */
static uint32_t s_last_sector;  /* offset of the record in use, 0 = none */

static uint32_t crc32(const void *data, size_t len)
{
    const uint8_t *p = data;
    uint32_t c = 0xFFFFFFFFu;
    while (len--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) {
            c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        }
    }
    return ~c;
}

static const record_t *read_rec(uint32_t offset)
{
    const record_t *r = (const record_t *)(XIP_BASE + offset);
    if (r->magic != REC_MAGIC || r->version != SETTINGS_VERSION || r->size != sizeof(settings_t)) {
        return NULL;
    }
    if (crc32(&r->s, sizeof r->s) != r->crc) {
        return NULL;
    }
    return r;
}

void settings_defaults(settings_t *s)
{
    memset(s, 0, sizeof *s);
    s->leverarm_m[0] = APP_GNSS_LEVERARM_X_M;
    s->leverarm_m[1] = APP_GNSS_LEVERARM_Y_M;
    s->leverarm_m[2] = APP_GNSS_LEVERARM_Z_M;
    s->acc_psd = APP_ACC_PSD;
    s->gyr_psd = APP_GYR_PSD;
    s->gyr_bias_rw = APP_GYR_BIAS_RW;
    s->acc_bias_rw = APP_ACC_BIAS_RW;
    s->automotive_min_speed = APP_AUTOMOTIVE_MIN_SPEED;
    s->baseline_yaw_deg = APP_BASELINE_YAW_DEG;
    s->baseline_gate_m = APP_BASELINE_GATE_M;
    s->antenna_dual = APP_GNSS_ANTENNA_MODE == APP_ANTENNA_DUAL;
    s->wifi_mode = APP_WIFI_MODE;
    snprintf(s->sta_ssid, sizeof s->sta_ssid, "%s", APP_WIFI_STA_SSID);
    snprintf(s->sta_password, sizeof s->sta_password, "%s", APP_WIFI_STA_PASSWORD);
}

bool settings_load(settings_t *s)
{
    const record_t *a = read_rec(SECTOR_A);
    const record_t *b = read_rec(SECTOR_B);
    const record_t *r = a;
    if (b && (!a || b->seq > a->seq)) {
        r = b;
    }
    if (!r) {
        settings_defaults(s);
        s_seq = 0;
        s_last_sector = 0;
        return false;
    }
    *s = r->s;
    /* Make sure the strings end, whatever is in flash. */
    s->sta_ssid[SETTINGS_SSID_MAX] = '\0';
    s->sta_password[SETTINGS_PW_MAX] = '\0';
    s_seq = r->seq;
    s_last_sector = r == a ? SECTOR_A : SECTOR_B;
    return true;
}

static bool in_range(float v, float lo, float hi) { return isfinite(v) && v >= lo && v <= hi; }

const char *settings_validate(const settings_t *s)
{
    for (int i = 0; i < 3; i++) {
        if (!in_range(s->leverarm_m[i], -10.0f, 10.0f)) {
            return "lever arm: each axis must be within -10 .. 10 m";
        }
    }
    if (!in_range(s->acc_psd, 1e-9f, 1e-1f)) {
        return "acc_psd: must be within 1e-9 .. 1e-1 (m/s^2)^2/Hz";
    }
    if (!in_range(s->gyr_psd, 1e-12f, 1e-3f)) {
        return "gyr_psd: must be within 1e-12 .. 1e-3 (rad/s)^2/Hz";
    }
    if (!in_range(s->gyr_bias_rw, 1e-9f, 1e-2f)) {
        return "gyr_bias_rw: must be within 1e-9 .. 1e-2 rad/s/sqrt(s)";
    }
    if (!in_range(s->acc_bias_rw, 1e-7f, 1e-1f)) {
        return "acc_bias_rw: must be within 1e-7 .. 1e-1 m/s^2/sqrt(s)";
    }
    if (!in_range(s->automotive_min_speed, 0.0f, 20.0f)) {
        return "automotive_min_speed: must be within 0 .. 20 m/s";
    }
    if (!in_range(s->baseline_yaw_deg, -180.0f, 360.0f)) {
        return "baseline_yaw_deg: must be within -180 .. 360";
    }
    if (!in_range(s->baseline_gate_m, 0.001f, 1.0f)) {
        return "baseline_gate_m: must be within 0.001 .. 1 m";
    }
    if (s->antenna_dual > 1) {
        return "antenna: must be single or dual";
    }
    if (s->wifi_mode > APP_WIFI_AP) {
        return "wifi_mode: must be auto, sta or ap";
    }
    size_t n = strnlen(s->sta_ssid, sizeof s->sta_ssid);
    if (n == 0 || n > SETTINGS_SSID_MAX) {
        return "sta_ssid: 1 .. 32 characters";
    }
    n = strnlen(s->sta_password, sizeof s->sta_password);
    if (n != 0 && (n < 8 || n > SETTINGS_PW_MAX)) {
        return "sta_password: empty (open network) or 8 .. 63 characters";
    }
    return NULL;
}

typedef struct {
    uint32_t offset;
    const record_t *rec; /* NULL = erase only */
} flash_job_t;

/* Runs with core 1 locked out and core 0's interrupts off. */
static void flash_job(void *param)
{
    const flash_job_t *j = param;
    flash_range_erase(j->offset, FLASH_SECTOR_SIZE);
    if (j->rec) {
        static uint8_t page[FLASH_PAGE_SIZE];
        memset(page, 0xFF, sizeof page);
        memcpy(page, j->rec, sizeof *j->rec);
        flash_range_program(j->offset, page, sizeof page);
    }
}

int settings_save(const settings_t *s)
{
    static record_t rec;
    memset(&rec, 0, sizeof rec);
    rec.magic = REC_MAGIC;
    rec.version = SETTINGS_VERSION;
    rec.size = sizeof(settings_t);
    rec.seq = s_seq + 1;
    rec.s = *s;
    rec.crc = crc32(&rec.s, sizeof rec.s);

    flash_job_t job = { .offset = s_last_sector == SECTOR_A ? SECTOR_B : SECTOR_A, .rec = &rec };
    int rc = flash_safe_execute(flash_job, &job, FLASH_TIMEOUT_MS);
    if (rc != 0) {
        return rc;
    }
    const record_t *back = read_rec(job.offset);
    if (!back || back->seq != rec.seq) {
        return -1; /* read-back failed */
    }
    s_seq = rec.seq;
    s_last_sector = job.offset;
    return 0;
}

int settings_erase(void)
{
    flash_job_t job = { .offset = SECTOR_A, .rec = NULL };
    int rc = flash_safe_execute(flash_job, &job, FLASH_TIMEOUT_MS);
    if (rc == 0) {
        job.offset = SECTOR_B;
        rc = flash_safe_execute(flash_job, &job, FLASH_TIMEOUT_MS);
    }
    if (rc == 0) {
        s_seq = 0;
        s_last_sector = 0;
    }
    return rc;
}

uint32_t settings_seq(void) { return s_seq; }

bool settings_reboot_fields_differ(const settings_t *a, const settings_t *b)
{
    return a->antenna_dual != b->antenna_dual || a->wifi_mode != b->wifi_mode ||
           strcmp(a->sta_ssid, b->sta_ssid) != 0 || strcmp(a->sta_password, b->sta_password) != 0;
}
