#include "config_web.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_config.h"
#include "jsonw.h"
#include "web.h"

/* Shared with the POST handler (lwIP context); touched by the main loop only
 * inside web_locked(). */
static settings_t s_base;          /* settings in use, the start of a save */
static config_cmd_t s_pending;     /* kind NONE = free */

static uint32_t s_result_seq;      /* main loop only */
static char s_result[160] = "none";

/* ---- form parsing (lwIP context) ------------------------------------------ */

static int hexval(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/* Decodes %XX and '+' in place over [s, s+n); returns the new length. */
static size_t url_decode(char *s, size_t n)
{
    size_t o = 0;
    for (size_t i = 0; i < n; i++) {
        if (s[i] == '+') {
            s[o++] = ' ';
        } else if (s[i] == '%' && i + 2 < n && hexval(s[i + 1]) >= 0 && hexval(s[i + 2]) >= 0) {
            s[o++] = (char)(hexval(s[i + 1]) * 16 + hexval(s[i + 2]));
            i += 2;
        } else {
            s[o++] = s[i];
        }
    }
    s[o] = '\0';
    return o;
}

typedef struct {
    char *buf;
    size_t len, pos;
} form_t;

/* Next key=value pair, both decoded and NUL-terminated in place. */
static bool form_next(form_t *f, char **key, char **val)
{
    if (f->pos >= f->len) {
        return false;
    }
    char *start = f->buf + f->pos;
    char *end = memchr(start, '&', f->len - f->pos);
    size_t n = end ? (size_t)(end - start) : f->len - f->pos;
    f->pos += n + 1;
    start[n] = '\0';
    char *eq = memchr(start, '=', n);
    if (eq == NULL) {
        *key = start;
        url_decode(start, n);
        *val = start + strlen(start); /* "" */
        return true;
    }
    *eq = '\0';
    url_decode(start, (size_t)(eq - start));
    url_decode(eq + 1, n - (size_t)(eq - start) - 1);
    *key = start;
    *val = eq + 1;
    return true;
}

static bool parse_float(const char *v, float *out)
{
    char *end;
    float x = strtof(v, &end);
    if (end == v) {
        return false;
    }
    while (*end == ' ') {
        end++;
    }
    if (*end != '\0') {
        return false;
    }
    *out = x;
    return true;
}

typedef struct {
    const char *key;
    size_t offset; /* into settings_t */
} float_field_t;

#define FF(k, m) { k, offsetof(settings_t, m) }
static const float_field_t k_floats[] = {
    FF("lever_x", leverarm_m[0]),
    FF("lever_y", leverarm_m[1]),
    FF("lever_z", leverarm_m[2]),
    FF("acc_psd", acc_psd),
    FF("gyr_psd", gyr_psd),
    FF("gyr_bias_rw", gyr_bias_rw),
    FF("acc_bias_rw", acc_bias_rw),
    FF("automotive_min_speed", automotive_min_speed),
    FF("baseline_yaw_deg", baseline_yaw_deg),
    FF("baseline_gate_m", baseline_gate_m),
};

/* Applies one form field to s. NULL, or an error message in err. */
static const char *apply_field(settings_t *s, const char *key, const char *val, char *err,
                               size_t cap)
{
    for (size_t i = 0; i < sizeof k_floats / sizeof k_floats[0]; i++) {
        if (strcmp(key, k_floats[i].key) == 0) {
            float *dst = (float *)((char *)s + k_floats[i].offset);
            if (!parse_float(val, dst)) {
                snprintf(err, cap, "%s: not a number", key);
                return err;
            }
            return NULL;
        }
    }
    if (strcmp(key, "antenna") == 0) {
        if (strcmp(val, "single") == 0) s->antenna_dual = 0;
        else if (strcmp(val, "dual") == 0) s->antenna_dual = 1;
        else { snprintf(err, cap, "antenna: single or dual"); return err; }
        return NULL;
    }
    if (strcmp(key, "wifi_mode") == 0) {
        if (strcmp(val, "auto") == 0) s->wifi_mode = APP_WIFI_AUTO;
        else if (strcmp(val, "sta") == 0) s->wifi_mode = APP_WIFI_STA;
        else if (strcmp(val, "ap") == 0) s->wifi_mode = APP_WIFI_AP;
        else { snprintf(err, cap, "wifi_mode: auto, sta or ap"); return err; }
        return NULL;
    }
    if (strcmp(key, "sta_ssid") == 0 || strcmp(key, "sta_password") == 0) {
        bool ssid = key[4] == 's';
        char *dst = ssid ? s->sta_ssid : s->sta_password;
        size_t max = ssid ? SETTINGS_SSID_MAX : SETTINGS_PW_MAX;
        if (strlen(val) > max) {
            snprintf(err, cap, "%s: at most %u characters", key, (unsigned)max);
            return err;
        }
        snprintf(dst, max + 1, "%s", val);
        return NULL;
    }
    if (strcmp(key, "action") == 0) {
        return NULL;
    }
    snprintf(err, cap, "unknown field: %.40s", key);
    return err;
}

static size_t reply(char *out, size_t cap, bool ok, const char *msg)
{
    char buf[200];
    jw_t w;
    jw_init(&w, buf, sizeof buf);
    jw_obj(&w, NULL);
    jw_bool(&w, "ok", ok);
    jw_str(&w, ok ? "queued" : "error", msg);
    jw_end(&w, '}');
    size_t n = w.len < cap ? w.len : cap;
    memcpy(out, buf, n);
    return n;
}

static void post_handler(const char *uri, const char *body, size_t len, char *out, size_t cap,
                         size_t *out_len)
{
    if (strcmp(uri, "/config") != 0) {
        *out_len = reply(out, cap, false, "unknown URI");
        return;
    }
    static char buf[WEB_POST_MAX + 1];
    memcpy(buf, body, len);
    buf[len] = '\0';

    /* First pass: the action. */
    config_cmd_kind_t kind = CFG_CMD_NONE;
    const char *action = "";
    form_t f = { buf, len, 0 };
    char *k, *v;
    static char copy[WEB_POST_MAX + 1];
    memcpy(copy, buf, len + 1);
    form_t f2 = { copy, len, 0 };
    while (form_next(&f2, &k, &v)) {
        if (strcmp(k, "action") == 0) {
            action = v;
        }
    }
    if (strcmp(action, "save") == 0) kind = CFG_CMD_SAVE;
    else if (strcmp(action, "defaults") == 0) kind = CFG_CMD_DEFAULTS;
    else if (strcmp(action, "reboot") == 0) kind = CFG_CMD_REBOOT;
    else if (strcmp(action, "reconfigure_gnss") == 0) kind = CFG_CMD_RECONFIGURE_GNSS;
    else if (strcmp(action, "reconfigure_imu") == 0) kind = CFG_CMD_RECONFIGURE_IMU;
    else {
        *out_len = reply(out, cap, false, "action must be save, defaults, reboot, "
                                          "reconfigure_gnss or reconfigure_imu");
        return;
    }
    if (s_pending.kind != CFG_CMD_NONE) {
        *out_len = reply(out, cap, false, "busy: the previous command is still running");
        return;
    }

    static config_cmd_t cmd;
    memset(&cmd, 0, sizeof cmd);
    cmd.kind = kind;
    if (kind == CFG_CMD_SAVE) {
        char err[96];
        cmd.settings = s_base;
        while (form_next(&f, &k, &v)) {
            if (apply_field(&cmd.settings, k, v, err, sizeof err) != NULL) {
                *out_len = reply(out, cap, false, err);
                return;
            }
        }
        const char *bad = settings_validate(&cmd.settings);
        if (bad != NULL) {
            *out_len = reply(out, cap, false, bad);
            return;
        }
    }
    s_pending = cmd;
    *out_len = reply(out, cap, true, action);
}

void config_web_init(void) { web_set_post_handler(post_handler); }

/* ---- main loop -------------------------------------------------------------- */

static void take_locked(void *arg)
{
    config_cmd_t *cmd = arg;
    *cmd = s_pending;
    s_pending.kind = CFG_CMD_NONE;
}

bool config_web_take(config_cmd_t *cmd)
{
    if (s_pending.kind == CFG_CMD_NONE) { /* cheap check without the lock */
        return false;
    }
    web_locked(take_locked, cmd);
    return cmd->kind != CFG_CMD_NONE;
}

static void set_base_locked(void *arg) { s_base = *(const settings_t *)arg; }

static const char *wifi_mode_name(uint8_t m)
{
    return m == APP_WIFI_AP ? "ap" : m == APP_WIFI_STA ? "sta" : "auto";
}

static void settings_obj(jw_t *w, const char *key, const settings_t *s)
{
    jw_obj(w, key);
    jw_num(w, "lever_x", (double)s->leverarm_m[0], 3);
    jw_num(w, "lever_y", (double)s->leverarm_m[1], 3);
    jw_num(w, "lever_z", (double)s->leverarm_m[2], 3);
    char t[24];
    /* %g keeps the small noise figures readable; jw_num would round them. */
    static const char *const k_noise[] = { "acc_psd", "gyr_psd", "gyr_bias_rw", "acc_bias_rw" };
    const float v[] = { s->acc_psd, s->gyr_psd, s->gyr_bias_rw, s->acc_bias_rw };
    for (int i = 0; i < 4; i++) {
        snprintf(t, sizeof t, "%g", (double)v[i]);
        jw_str(w, k_noise[i], t);
    }
    jw_num(w, "automotive_min_speed", (double)s->automotive_min_speed, 2);
    jw_num(w, "baseline_yaw_deg", (double)s->baseline_yaw_deg, 2);
    jw_num(w, "baseline_gate_m", (double)s->baseline_gate_m, 3);
    jw_str(w, "antenna", s->antenna_dual ? "dual" : "single");
    jw_str(w, "wifi_mode", wifi_mode_name(s->wifi_mode));
    jw_str(w, "sta_ssid", s->sta_ssid);
    jw_bool(w, "sta_password_set", s->sta_password[0] != '\0');
    jw_end(w, '}');
}

void config_web_publish(const settings_t *cur, const settings_t *boot, bool from_flash,
                        const char *result)
{
    static char buf[WEB_DOC_MAX];
    static settings_t defaults;
    jw_t w;

    web_locked(set_base_locked, (void *)cur);
    if (result != NULL) {
        snprintf(s_result, sizeof s_result, "%s", result);
        s_result_seq++;
    }
    settings_defaults(&defaults);

    jw_init(&w, buf, sizeof buf);
    jw_obj(&w, NULL);
    jw_str(&w, "source", from_flash ? "flash" : "defaults");
    jw_int(&w, "seq", settings_seq());
    settings_obj(&w, "settings", cur);
    settings_obj(&w, "defaults", &defaults);
    jw_arr(&w, "reboot_pending");
    if (cur->antenna_dual != boot->antenna_dual) jw_str(&w, NULL, "antenna");
    if (cur->wifi_mode != boot->wifi_mode) jw_str(&w, NULL, "wifi_mode");
    if (strcmp(cur->sta_ssid, boot->sta_ssid) != 0) jw_str(&w, NULL, "sta_ssid");
    if (strcmp(cur->sta_password, boot->sta_password) != 0) jw_str(&w, NULL, "sta_password");
    jw_end(&w, ']');
    jw_obj(&w, "last_result");
    jw_int(&w, "seq", s_result_seq);
    jw_str(&w, "text", s_result);
    jw_end(&w, '}');
    jw_end(&w, '}');
    if (jw_ok(&w)) {
        web_set_doc(WEB_DOC_CONFIG, buf, w.len);
    }
}
