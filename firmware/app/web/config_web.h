/*
 * Web config API (plans/web-config-server.md, Phase 4).
 *
 * GET  /config.json  the settings in use, the build defaults, which fields
 *                    wait for a reboot, and the result of the last command.
 * POST /config       form-encoded (application/x-www-form-urlencoded):
 *                    action=save|defaults|reboot|reconfigure_gnss|reconfigure_imu,
 *                    and for save the fields named in config.html. A field
 *                    left out keeps its current value; sta_password is only
 *                    changed when sent.
 *
 * The POST handler runs in lwIP context: it parses and range-checks a save
 * and queues one command. The main loop takes it with config_web_take(),
 * writes flash, applies it, and reports through config_web_publish().
 */
#ifndef CONFIG_WEB_H
#define CONFIG_WEB_H

#include <stdbool.h>
#include <stdint.h>

#include "settings.h"

typedef enum {
    CFG_CMD_NONE = 0,
    CFG_CMD_SAVE,
    CFG_CMD_DEFAULTS,
    CFG_CMD_REBOOT,
    CFG_CMD_RECONFIGURE_GNSS,
    CFG_CMD_RECONFIGURE_IMU,
} config_cmd_kind_t;

typedef struct {
    config_cmd_kind_t kind;
    settings_t settings; /* CFG_CMD_SAVE */
} config_cmd_t;

/* Registers the POST handler with the web server. */
void config_web_init(void);

/* Main loop: the queued command, if any. */
bool config_web_take(config_cmd_t *cmd);

/* Main loop: rebuild /config.json. cur = settings in use, boot = what this
 * boot started with, result = text of the last command's outcome (NULL keeps
 * the previous one, and its counter). */
void config_web_publish(const settings_t *cur, const settings_t *boot, bool from_flash,
                        const char *result);

#endif /* CONFIG_WEB_H */
