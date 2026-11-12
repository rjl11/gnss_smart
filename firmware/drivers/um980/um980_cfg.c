/*
 * UM980 / UM982 boot state machine: detect, identify (and pick the antenna
 * mode), check, provision. Follows plans/gnss-um98x-config.md. CONFIG RXTYPE
 * is never sent.
 *
 * Reply formats that are not yet confirmed on a UM980 (the plan's open bench
 * questions) are matched loosely, and with cfg.verbose every reply line is
 * printed while the state machine is not in RUN.
 */
#include "um980.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/time.h"

/* Timing. */
#define VERSION_WAIT_US         500000u
#define DETECT_RETRY_US         5000000u
#define QUERY_TIMEOUT_US        5000000u
#define QUERY_SILENCE_US        300000u
#define CMD_RETRIES             3u
#define PROBE_PERIOD_US         1000000u
#define PROVE_WINDOW_US         2000000u
#define RUN_SILENCE_US          5000000u
#define MAX_PROVISION_ATTEMPTS  2u

static const unsigned k_bauds[] = {
    460800, 115200, 921600, 230400, 57600, 38400, 19200, 9600,
};
#define N_BAUDS (sizeof k_bauds / sizeof k_bauds[0])

/* Target configuration, as it appears in the CONFIG reply after "CONFIG ".
 * The signal group and the heading mode depend on the antenna mode, so they
 * are filled in by cfg_target(). */
enum {
    CFG_COM1 = 0,
    CFG_COM2,
    CFG_COM3,
    CFG_PPS,
    CFG_SIGNALGROUP,
    CFG_PSRVELDRPOS,
    CFG_PVTALG,
    CFG_HEADING,
    CFG_COUNT
};
static const char *const k_cfg_fixed[CFG_COUNT] = {
    [CFG_COM1] = "COM1 460800",
    [CFG_COM2] = "COM2 460800",
    [CFG_COM3] = "COM3 460800",
    [CFG_PPS] = "PPS ENABLE GPS POSITIVE 100000 1000 0 0",
    [CFG_PSRVELDRPOS] = "PSRVELDRPOS DISABLE",
    [CFG_PVTALG] = "PVTALG MULTI",
    [CFG_HEADING] = "HEADING VARIABLELENGTH",
};
#define CFG_BIT(i)              (1u << (i))
/* PSRVELDRPOS may not be listed by CONFIG (seen on a UM982), so its absence
 * is not a mismatch. The command is then sent again on entering RUN.
 * PVTALG is listed once it has been set (seen on a UM980, 2026-11-12), so it
 * is required: a receiver still on its default (SINGLE on the UM982) gets
 * provisioned. */
#define CFG_REQUIRED_COMMON     (CFG_BIT(CFG_COM1) | CFG_BIT(CFG_COM2) | CFG_BIT(CFG_COM3) | \
                                 CFG_BIT(CFG_PPS) | CFG_BIT(CFG_SIGNALGROUP) | CFG_BIT(CFG_PVTALG))

/* The heading modes of CONFIG HEADING (manual §4.18). Other HEADING lines
 * (RELIABILITY, LENGTH, OFFSET) are not compared. */
static const char *const k_heading_modes[] = {
    "FIXLENGTH", "VARIABLELENGTH", "STATIC", "LOWDYNAMIC", "TRACTOR",
};

#define LOG_BIT_BESTNAV         (1u << 0)
#define LOG_BIT_STADOP          (1u << 1)
#define LOG_BIT_UNIHEADING      (1u << 2)

/* Provisioning script. */
typedef enum {
    OP_CMD,       /* send, need "response: OK"; retried                      */
    OP_CMD_OPT,   /* send once, wait for a reply but go on without one       */
    OP_WAIT,      /* wait ms                                                 */
    OP_BAUD,      /* switch UART1 to the baud in ms                          */
    OP_PROBE,     /* VERSIONB every second until answered; ms = time limit   */
    OP_CHECK_SG,  /* CONFIG must show the mode's signal group                */
    OP_END
} op_t;

/* A NULL text on an OP_CMD_OPT step means the mode's signal-group command. */
typedef struct {
    op_t        op;
    const char *text;
    uint32_t    ms;
    bool        dual_only; /* skipped unless the effective mode is dual      */
} script_step_t;

static const script_step_t k_script[] = {
    /* 1. Back to factory state. FRESET puts every port at 115200. */
    { OP_CMD_OPT,  "UNLOG",                                            1000, false },
    { OP_CMD_OPT,  "FRESET",                                           1000, false },
    { OP_BAUD,     NULL,                                             115200, false },
    { OP_WAIT,     NULL,                                              15000, false },
    { OP_PROBE,    NULL,                                              60000, false },
    /* 2. Signal group first: it reboots the receiver if the value changes. */
    { OP_CMD_OPT,  NULL,                                               5000, false },
    { OP_WAIT,     NULL,                                              15000, false },
    { OP_PROBE,    NULL,                                              60000, false },
    { OP_CHECK_SG, NULL,                                                  0, false },
    /* 3. Settings. */
    { OP_CMD,      "UNLOG COM1",                                       5000, false },
    { OP_CMD,      "UNLOG COM2",                                       5000, false },
    { OP_CMD,      "UNLOG COM3",                                       5000, false },
    { OP_CMD,      "MODE ROVER SURVEY",                                5000, false },
    { OP_CMD,      "CONFIG PSRVELDRPOS DISABLE",                       5000, false },
    { OP_CMD,      "CONFIG PVTALG MULTI",                              5000, false },
    { OP_CMD,      "CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0",   5000, false },
    { OP_CMD,      "CONFIG HEADING VARIABLELENGTH",                    5000, true  },
    /* 4. Baud rates, our own port last. Its reply may come at either rate. */
    { OP_CMD,      "CONFIG COM2 460800",                               5000, false },
    { OP_CMD,      "CONFIG COM3 460800",                               5000, false },
    { OP_CMD_OPT,  "CONFIG COM1 460800",                               1000, false },
    { OP_BAUD,     NULL,                                             460800, false },
    { OP_WAIT,     NULL,                                                100, false },
    { OP_PROBE,    NULL,                                              10000, false },
    /* 5. Logs, then save. */
    { OP_CMD,      "BESTNAVB COM1 0.05",                               5000, false },
    { OP_CMD,      "STADOPB COM1 1",                                   5000, false },
    { OP_CMD,      "UNIHEADINGB COM1 0.05",                            5000, true  },
    { OP_CMD,      "SAVECONFIG",                                      10000, false },
    /* 6. Prove that it was saved: restart, then detect and check again. */
    { OP_CMD_OPT,  "RESET",                                            1000, false },
    { OP_WAIT,     NULL,                                              15000, false },
    { OP_END,      NULL,                                                  0, false },
};

/* The CONFIG reply text this mode needs for item i, or NULL if the item is
 * not compared in this mode. */
static const char *cfg_target(const um980_t *g, unsigned i)
{
    if (i == CFG_SIGNALGROUP) {
        return g->sg_cmd != NULL ? g->sg_cmd + 7 : NULL; /* after "CONFIG " */
    }
    if (i == CFG_HEADING && !g->dual) {
        return NULL;
    }
    return k_cfg_fixed[i];
}

static uint32_t cfg_required(const um980_t *g)
{
    return CFG_REQUIRED_COMMON | (g->dual ? CFG_BIT(CFG_HEADING) : 0u);
}

/* Combines the antenna-mode setting with the receiver found
 * (plans/gnss-um98x-config.md, "Antenna mode" and "Target configuration").
 * Returns false for a receiver this driver can't configure. */
static bool select_mode(um980_t *g)
{
    if (g->model == UM980_PRODUCT_UM980 && g->build >= UM980_MIN_BUILD) {
        g->dual = false;
        g->dual_refused = g->cfg.dual_antenna;
        g->sg_cmd = "CONFIG SIGNALGROUP 2";
        return true;
    }
    if (g->model == UM980_PRODUCT_UM982 && g->build >= UM982_MIN_BUILD) {
        g->dual = g->cfg.dual_antenna;
        g->dual_refused = false;
        /* Single mode uses the dual group too and leaves the heading unused:
         * the UM982 refuses "2 0" (bench results 2026-11-12). It accepts
         * only 3 6, 4 5, 5 0 and 7 0. */
        if (g->build >= UM982_SG36_MIN_BUILD) {
            g->sg_cmd = "CONFIG SIGNALGROUP 3 6";
        } else {
            g->sg_cmd = "CONFIG SIGNALGROUP 4 5";
        }
        return true;
    }
    g->dual = false;
    g->dual_refused = false;
    g->sg_cmd = NULL;
    return false;
}

#define LOG(g, ...) do { if ((g)->cfg.verbose) { printf("[um980] " __VA_ARGS__); printf("\n"); } } while (0)

/* ---- Reply parsing ----------------------------------------------------- */

static bool starts_with(const char *s, const char *prefix)
{
    return strncmp(s, prefix, strlen(prefix)) == 0;
}

/* "$CONFIG,<item>,<command>*cs": compare <command> with the target table. */
static void on_config_line(um980_t *g, const char *line)
{
    const char *p = strchr(line + 8, ',');
    if (p == NULL) {
        return;
    }
    p++;
    if (starts_with(p, "CONFIG ")) {
        p += 7;
    }
    if (starts_with(p, "HEADING ")) {
        bool is_mode = false;
        for (unsigned k = 0; k < sizeof k_heading_modes / sizeof k_heading_modes[0]; k++) {
            if (starts_with(p + 8, k_heading_modes[k])) {
                is_mode = true;
            }
        }
        if (!is_mode) {
            return; /* RELIABILITY, LENGTH, OFFSET: not part of the target */
        }
    }
    for (unsigned i = 0; i < CFG_COUNT; i++) {
        const char *target = cfg_target(g, i);
        if (target == NULL) {
            continue;
        }
        size_t key_len = (size_t)(strchr(target, ' ') - target) + 1u; /* "COM1 " */
        if (strncmp(p, target, key_len) != 0) {
            continue;
        }
        size_t n = strlen(target);
        /* A signal group must match in full: "SIGNALGROUP 3 6" is not
         * "SIGNALGROUP 3". */
        bool end_ok = p[n] == '\0' || p[n] == '*' || (p[n] == ' ' && i != CFG_SIGNALGROUP);
        if (strncmp(p, target, n) == 0 && end_ok) {
            g->cfg_seen |= CFG_BIT(i);
        } else {
            g->cfg_bad = true;
            LOG(g, "config differs: \"%s\", want \"%s\"", p, target);
        }
        if (i == CFG_PSRVELDRPOS) {
            g->psrvel_listed = true;
        }
    }
}

/* UNILOGLIST: "<3" then "< BESTNAVB COM1 0.05" per log. The period is the
 * last token, so a trigger word in between does not matter. */
static void on_loglist_line(um980_t *g, const char *line)
{
    char buf[64];
    char *tok[5];
    int n = 0;

    strncpy(buf, line + 1, sizeof buf - 1);
    buf[sizeof buf - 1] = '\0';
    for (char *t = strtok(buf, " \t"); t != NULL && n < 5; t = strtok(NULL, " \t")) {
        tok[n++] = t;
    }
    if (n == 0) {
        return;
    }
    if (g->log_count < 0) {
        g->log_count = atoi(tok[0]);
        return;
    }
    g->log_lines++;
    if (n < 3) {
        g->log_bad = true;
        return;
    }
    float period = strtof(tok[n - 1], NULL);
    if (strcmp(tok[0], "BESTNAVB") == 0 && strcmp(tok[1], "COM1") == 0 &&
        period > 0.049f && period < 0.051f) {
        g->log_seen |= LOG_BIT_BESTNAV;
    } else if (strcmp(tok[0], "STADOPB") == 0 && strcmp(tok[1], "COM1") == 0 &&
               period > 0.99f && period < 1.01f) {
        g->log_seen |= LOG_BIT_STADOP;
    } else if (g->dual && strcmp(tok[0], "UNIHEADINGB") == 0 && strcmp(tok[1], "COM1") == 0 &&
               period > 0.049f && period < 0.051f) {
        g->log_seen |= LOG_BIT_UNIHEADING;
    } else {
        g->log_bad = true;
        LOG(g, "unexpected log: \"%s\"", line);
    }
}

void um980_cfg_on_line(um980_t *g, const char *line, uint64_t now)
{
    g->last_line_us = now;
    g->reply_seen_ok = true; /* something answered */

    if (g->cfg.verbose && g->state != UM980_ST_RUN) {
        printf("[um980] < %s\n", line);
    }

    if (starts_with(line, "$command,")) {
        g->reply = strstr(line, "response: OK") != NULL ? 1 : -1;
    } else if (starts_with(line, "$CONFIG,")) {
        on_config_line(g, line);
    } else if (starts_with(line, "#MODE,")) {
        g->mode_ok = strstr(line, "MODE ROVER SURVEY") != NULL;
    } else if (line[0] == '<') {
        on_loglist_line(g, line);
    }
}

void um980_cfg_on_version(um980_t *g, uint32_t product, const char *sw, const char *psn)
{
    strncpy(g->sw_version, sw, sizeof g->sw_version - 1);
    strncpy(g->serial, psn, sizeof g->serial - 1);
    const char *b = strstr(sw, "Build");
    g->build = b != NULL ? (uint32_t)strtoul(b + 5, NULL, 10) : 0;
    g->model = (int)product;
    g->got_version = true;
}

/* ---- Steps ------------------------------------------------------------- */

static void enter(um980_t *g, um980_state_t s)
{
    if (g->state != s) {
        LOG(g, "%s -> %s", um980_state_name(g->state), um980_state_name(s));
    }
    g->state = s;
    g->step = 0;
    g->retries = 0;
    g->script_pc = 0;
}

static void enter_detect(um980_t *g, uint64_t now, uint32_t delay_us)
{
    enter(g, UM980_ST_DETECT);
    g->baud_idx = 0;
    g->deadline_us = now + delay_us;
}

static void enter_run(um980_t *g, uint64_t now)
{
    enter(g, UM980_ST_RUN);
    g->last_line_us = now;
    g->win_nav0 = g->stats.frames;
    g->deadline_us = now + RUN_SILENCE_US;
    if (!g->degraded) {
        g->last_error = UM980_OK;
        if (!g->psrvel_listed) {
            /* Not shown by CONFIG, so its state is unknown: set it (RAM only). */
            um980_send_line(g, "CONFIG PSRVELDRPOS DISABLE");
        }
    }
}

/* A provisioning run or its proof failed. After two failures the receiver is
 * used as it is, so a problem cannot turn into an endless FRESET loop. */
static void provision_failed(um980_t *g, uint64_t now, int err)
{
    g->last_error = err;
    g->proving = false;
    LOG(g, "error: %s (attempt %u of %u)", um980_strerror(err), g->provision_attempts,
        MAX_PROVISION_ATTEMPTS);
    enter_detect(g, now, DETECT_RETRY_US);
}

static void enter_provision(um980_t *g, uint64_t now)
{
    if (g->provision_attempts >= MAX_PROVISION_ATTEMPTS) {
        g->degraded = true;
        LOG(g, "provisioning failed %u times: running with the receiver as it is",
            MAX_PROVISION_ATTEMPTS);
        enter_run(g, now);
        return;
    }
    g->provision_attempts++;
    g->proving = false;
    enter(g, UM980_ST_PROVISION);
}

/* Sends a command and waits for its "$command" reply. Returns 1 on OK, -1 on
 * an error reply or no reply after the retries, 0 while waiting. */
static int cmd_step(um980_t *g, uint64_t now, const char *cmd, uint32_t timeout_ms, unsigned tries)
{
    if (g->step == 0) {
        g->reply = 0;
        LOG(g, "> %s", cmd);
        um980_send_line(g, cmd);
        g->step_deadline_us = now + (uint64_t)timeout_ms * 1000u;
        g->step = 1;
        return 0;
    }
    if (g->reply > 0) {
        return 1;
    }
    if (g->reply < 0 || now >= g->step_deadline_us) {
        if (++g->retries >= tries) {
            return -1;
        }
        g->step = 0;
    }
    return 0;
}

/* Sends a query and collects its reply lines until they stop. Returns 1 when
 * something answered, -1 on silence, 0 while waiting. */
static int query_step(um980_t *g, uint64_t now, const char *cmd)
{
    if (g->step == 0) {
        g->reply = 0;
        g->reply_seen_ok = false;
        LOG(g, "> %s", cmd);
        um980_send_line(g, cmd);
        g->step_deadline_us = now + QUERY_TIMEOUT_US;
        g->step = 1;
        return 0;
    }
    if (g->reply_seen_ok && now - g->last_line_us > QUERY_SILENCE_US) {
        return 1;
    }
    if (now >= g->step_deadline_us) {
        return g->reply_seen_ok ? 1 : -1;
    }
    return 0;
}

/* Sends VERSIONB every second until a VERSION frame arrives. Returns 1 when
 * answered, -1 after limit_ms, 0 while waiting. */
static int probe_step(um980_t *g, uint64_t now, uint32_t limit_ms)
{
    if (g->step == 0) {
        g->got_version = false;
        g->deadline_us = now + (uint64_t)limit_ms * 1000u;
        g->step_deadline_us = now;
        g->step = 1;
    }
    if (g->got_version) {
        return 1;
    }
    if (now >= g->deadline_us) {
        return -1;
    }
    if (now >= g->step_deadline_us) {
        um980_send_line(g, "VERSIONB");
        g->step_deadline_us = now + PROBE_PERIOD_US;
    }
    return 0;
}

static void reset_check(um980_t *g)
{
    g->cfg_seen = 0;
    g->cfg_bad = false;
    g->psrvel_listed = false;
    g->mode_ok = false;
    g->log_count = -1;
    g->log_lines = 0;
    g->log_seen = 0;
    g->log_bad = false;
}

static bool check_passed(um980_t *g)
{
    uint32_t need = cfg_required(g);
    uint32_t want_logs = LOG_BIT_BESTNAV | LOG_BIT_STADOP | (g->dual ? LOG_BIT_UNIHEADING : 0u);
    int n_logs = g->dual ? 3 : 2;
    bool cfg_ok = (g->cfg_seen & need) == need && !g->cfg_bad;
    bool logs_ok = g->log_count == n_logs && g->log_lines == n_logs && !g->log_bad &&
                   g->log_seen == want_logs;
    if (!cfg_ok) {
        LOG(g, "check: CONFIG differs (items seen 0x%02lx, want 0x%02lx)",
            (unsigned long)g->cfg_seen, (unsigned long)need);
    }
    if (!g->mode_ok) {
        LOG(g, "check: MODE is not ROVER SURVEY");
    }
    if (!logs_ok) {
        LOG(g, "check: logs differ (count %d, seen 0x%lx)", g->log_count, (unsigned long)g->log_seen);
    }
    return cfg_ok && g->mode_ok && logs_ok;
}

static void next_script_step(um980_t *g)
{
    g->script_pc++;
    g->step = 0;
    g->retries = 0;
}

static void provision_step(um980_t *g, uint64_t now)
{
    const script_step_t *s = &k_script[g->script_pc];
    int r;

    if (s->dual_only && !g->dual) {
        next_script_step(g);
        return;
    }

    switch (s->op) {
    case OP_CMD:
        r = cmd_step(g, now, s->text, s->ms, CMD_RETRIES);
        if (r > 0) {
            next_script_step(g);
        } else if (r < 0) {
            LOG(g, "command failed: %s", s->text);
            provision_failed(g, now, UM980_E_COMMAND);
        }
        break;

    case OP_CMD_OPT:
        if (cmd_step(g, now, s->text != NULL ? s->text : g->sg_cmd, s->ms, 1) != 0) {
            next_script_step(g);
        }
        break;

    case OP_WAIT:
        if (g->step == 0) {
            g->step_deadline_us = now + (uint64_t)s->ms * 1000u;
            g->step = 1;
        } else if (now >= g->step_deadline_us) {
            next_script_step(g);
        }
        break;

    case OP_BAUD:
        if (uart_dma_tx_idle(g->port)) {
            g->baud = s->ms;
            uart_dma_set_baud(g->port, g->baud);
            g->rx_mode = 0;
            LOG(g, "UART at %u baud", g->baud);
            next_script_step(g);
        }
        break;

    case OP_PROBE:
        r = probe_step(g, now, s->ms);
        if (r > 0) {
            next_script_step(g);
        } else if (r < 0) {
            provision_failed(g, now, UM980_E_NO_REBOOT);
        }
        break;

    case OP_CHECK_SG:
        if (g->step == 0) {
            reset_check(g);
        }
        r = query_step(g, now, "CONFIG");
        if (r != 0) {
            if (r > 0 && (g->cfg_seen & CFG_BIT(CFG_SIGNALGROUP)) != 0) {
                next_script_step(g);
            } else {
                provision_failed(g, now, UM980_E_SIGNALGROUP);
            }
        }
        break;

    case OP_END:
    default:
        g->proving = true;
        enter_detect(g, now, 0);
        break;
    }
}

void um980_cfg_step(um980_t *g, uint64_t now)
{
    int r;

    switch (g->state) {
    case UM980_ST_DETECT:
        if (g->step == 0) { /* pause before a scan */
            if (now >= g->deadline_us) {
                g->step = 1;
            }
        } else if (g->step == 1) { /* try the next baud */
            if (!uart_dma_tx_idle(g->port)) {
                break;
            }
            g->baud = k_bauds[g->baud_idx];
            uart_dma_set_baud(g->port, g->baud);
            g->rx_mode = 0;
            g->got_version = false;
            um980_send_line(g, "VERSIONB");
            g->step_deadline_us = now + VERSION_WAIT_US;
            g->step = 2;
        } else if (g->got_version) {
            LOG(g, "found at %u baud: product %d, %s, %s", g->baud, g->model, g->sw_version,
                g->serial);
            if (select_mode(g)) {
                LOG(g, "%s, %s antenna, target %s",
                    g->model == UM980_PRODUCT_UM982 ? "UM982" : "UM980",
                    g->dual ? "dual" : "single", g->sg_cmd);
                if (g->dual_refused) {
                    LOG(g, "warning: dual antenna is set, but a UM980 is fitted: using single");
                }
            }
            if (g->sg_cmd == NULL) {
                /* Not a receiver this driver knows: nothing is written to
                 * it, it is used as it is. */
                g->last_error = UM980_E_UNSUPPORTED;
                LOG(g, "error: %s, running with the receiver as it is",
                    um980_strerror(UM980_E_UNSUPPORTED));
                g->degraded = true;
                enter_run(g, now);
            } else if (g->baud == UM980_TARGET_BAUD) {
                enter(g, UM980_ST_CHECK);
            } else if (g->proving) {
                provision_failed(g, now, UM980_E_NOT_SAVED);
            } else {
                enter_provision(g, now);
            }
        } else if (now >= g->step_deadline_us) {
            if (++g->baud_idx >= N_BAUDS) {
                if (g->proving) {
                    provision_failed(g, now, UM980_E_NO_REBOOT);
                } else {
                    g->last_error = UM980_E_NO_RECEIVER;
                    LOG(g, "error: %s, retrying", um980_strerror(UM980_E_NO_RECEIVER));
                    enter_detect(g, now, DETECT_RETRY_US);
                }
            } else {
                g->step = 1;
            }
        }
        break;

    case UM980_ST_CHECK: {
        static const char *const k_queries[] = { "CONFIG", "MODE", "UNILOGLIST" };
        if (g->script_pc == 0 && g->step == 0) {
            reset_check(g);
        }
        r = query_step(g, now, k_queries[g->script_pc]);
        if (r == 0) {
            break;
        }
        if (r > 0 && g->script_pc + 1u < sizeof k_queries / sizeof k_queries[0]) {
            g->script_pc++;
            g->step = 0;
            break;
        }
        if (r > 0 && check_passed(g)) {
            if (g->proving) {
                enter(g, UM980_ST_PROVE_RATE);
                g->win_nav0 = g->stats.nav_epochs;
                g->deadline_us = now + PROVE_WINDOW_US;
            } else {
                LOG(g, "configuration already matches, nothing written");
                enter_run(g, now);
            }
        } else if (g->proving) {
            provision_failed(g, now, UM980_E_NOT_SAVED);
        } else {
            enter_provision(g, now);
        }
        break;
    }

    case UM980_ST_PROVISION:
        provision_step(g, now);
        break;

    case UM980_ST_PROVE_RATE:
        if (now < g->deadline_us) {
            break;
        }
        {
            uint32_t n = g->stats.nav_epochs - g->win_nav0;
            if (n >= 38 && n <= 42) { /* 20 +/- 1 Hz over 2 s */
                g->proving = false;
                g->provision_attempts = 0;
                g->stats.provisions++;
                LOG(g, "provisioned and saved, BESTNAVB at %lu Hz", (unsigned long)(n / 2u));
                enter_run(g, now);
            } else {
                LOG(g, "BESTNAVB: %lu frames in 2 s", (unsigned long)n);
                provision_failed(g, now, UM980_E_RATE);
            }
        }
        break;

    case UM980_ST_RUN:
    default:
        /* A receiver that stops sending was unplugged or restarted. */
        if (g->stats.frames != g->win_nav0) {
            g->win_nav0 = g->stats.frames;
            g->deadline_us = now + RUN_SILENCE_US;
        } else if (now >= g->deadline_us && !g->degraded) {
            LOG(g, "no data for 5 s, detecting again");
            enter_detect(g, now, 0);
        }
        break;
    }
}

void um980_cfg_init(um980_t *g, uint64_t now)
{
    g->state = UM980_ST_DETECT;
    g->log_count = -1;
    enter_detect(g, now, 0);
}

void um980_reconfigure(um980_t *g)
{
    g->provision_attempts = 0;
    g->degraded = false;
    if (g->sg_cmd == NULL) {
        /* Not identified yet: the target table is unknown. Detection
         * provisions the receiver if its settings differ. */
        enter_detect(g, time_us_64(), 0);
        return;
    }
    enter_provision(g, time_us_64());
}
