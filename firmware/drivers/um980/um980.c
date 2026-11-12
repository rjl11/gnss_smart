/*
 * UM980 driver: receive framer, binary log decoding and PPS time-tagging.
 * Message layouts are from the Unicore N4 commands manual (docs/), sections
 * 7.5.1 (header), 7.5.2 (VERSION), 7.5.21 (BESTNAV) and 7.5.96 (STADOP).
 */
#include "um980.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "pico/time.h"

#define HEADER_LEN          24u
#define CRC_LEN             4u
#define OFS_MSG_ID          4
#define OFS_MSG_LEN         6
#define OFS_WEEK            10
#define OFS_TOW_MS          12
#define OFS_LEAP            21

#define VERSION_MIN_LEN     (4u + 33u + 129u + 66u)
#define STADOP_MIN_LEN      40u
#define BESTNAV_MIN_LEN     120u

#define MODE_IDLE           0
#define MODE_BINARY         1
#define MODE_ASCII          2

/* A PPS edge older than this is not used to time-tag an epoch. */
#define PPS_MAX_AGE_US      1500000
/* The time from an epoch's validity to its arrival must be in this range. */
#define NAV_MAX_LATENCY_US  500000

static uint32_t s_crc_table[256];

static volatile uint64_t s_pps_t_us;
static volatile uint32_t s_pps_count;
static int s_pps_pin = -1;

/* ---- Helpers ----------------------------------------------------------- */

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static float rd_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, sizeof f);
    return f;
}

static double rd_f64(const uint8_t *p)
{
    double d;
    memcpy(&d, p, sizeof d);
    return d;
}

/* CRC-32 as in the manual's CalculateCRC32(): reflected 0xEDB88320, start
 * value 0, no final inversion. */
static void crc_table_init(void)
{
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : c >> 1;
        }
        s_crc_table[i] = c;
    }
}

static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t c = 0;
    while (n--) {
        c = s_crc_table[(c ^ *p++) & 0xFFu] ^ (c >> 8);
    }
    return c;
}

/* ---- PPS --------------------------------------------------------------- */

static void pps_irq(void)
{
    if (gpio_get_irq_event_mask((unsigned)s_pps_pin) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq((unsigned)s_pps_pin, GPIO_IRQ_EDGE_RISE);
        s_pps_t_us = time_us_64();
        s_pps_count++;
    }
}

static void pps_attach(int pin)
{
    s_pps_pin = pin;
    gpio_init((unsigned)pin);
    gpio_set_dir((unsigned)pin, GPIO_IN);
    gpio_pull_down((unsigned)pin); /* no edges from an unconnected pin */
    gpio_acknowledge_irq((unsigned)pin, GPIO_IRQ_EDGE_RISE);
    gpio_add_raw_irq_handler((unsigned)pin, pps_irq);
    gpio_set_irq_enabled((unsigned)pin, GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

static uint32_t pps_read(uint64_t *t_us)
{
    uint32_t save = save_and_disable_interrupts();
    *t_us = s_pps_t_us;
    uint32_t n = s_pps_count;
    restore_interrupts(save);
    return n;
}

/* The PPS edge marks a whole GPS second. The frame's time of week says how
 * far after that second the epoch is valid. */
static void time_tag(um980_nav_t *n)
{
    uint64_t t_pps;

    n->t_valid_us = n->t_arrival_us;
    n->t_from_pps = false;

    if (s_pps_pin < 0 || pps_read(&t_pps) == 0) {
        return;
    }
    int64_t since_pps_us = (int64_t)n->t_arrival_us - (int64_t)t_pps;
    if (since_pps_us < 0 || since_pps_us > PPS_MAX_AGE_US) {
        return;
    }
    /* tow - (arrival - pps) is the PPS second minus the output latency, so
     * rounding to the nearest second gives the PPS second for a latency
     * below 500 ms. */
    int64_t est_ms = (int64_t)n->tow_ms - since_pps_us / 1000;
    int64_t q = est_ms + 500;
    int64_t pps_tow_ms = q - (((q % 1000) + 1000) % 1000); /* floor to a second */
    int64_t t_valid = (int64_t)t_pps + ((int64_t)n->tow_ms - pps_tow_ms) * 1000;
    int64_t latency = (int64_t)n->t_arrival_us - t_valid;
    if (latency < 0 || latency > NAV_MAX_LATENCY_US) {
        return;
    }
    n->t_valid_us = (uint64_t)t_valid;
    n->t_from_pps = true;
}

/* ---- Frames ------------------------------------------------------------ */

static bool is_fix_type(uint32_t type)
{
    switch (type) {
    case UM980_TYPE_SINGLE:
    case UM980_TYPE_PSRDIFF:
    case UM980_TYPE_SBAS:
    case UM980_TYPE_L1_FLOAT:
    case UM980_TYPE_IONOFREE_FLOAT:
    case UM980_TYPE_NARROW_FLOAT:
    case UM980_TYPE_L1_INT:
    case UM980_TYPE_WIDE_INT:
    case UM980_TYPE_NARROW_INT:
        return true;
    default:
        return type >= UM980_TYPE_PPP_CONVERGING && type <= UM980_TYPE_PPP_RTK;
    }
}

static bool sigma_ok(float s) { return isfinite(s) && s > 0.0f; }

static void decode_bestnav(um980_t *g, const uint8_t *p, um980_nav_t *n)
{
    memset(n, 0, sizeof *n);
    n->t_arrival_us = g->port->rx_snap_us;
    n->week = rd_u16(&g->rx[OFS_WEEK]);
    n->tow_ms = rd_u32(&g->rx[OFS_TOW_MS]);
    n->leap_s = g->rx[OFS_LEAP];

    n->sol_status = rd_u32(p + 0);
    n->pos_type = rd_u32(p + 4);
    n->lat_deg = rd_f64(p + 8);
    n->lon_deg = rd_f64(p + 16);
    n->hgt_msl_m = rd_f64(p + 24);
    n->undulation_m = rd_f32(p + 32);
    n->sd_lat_m = rd_f32(p + 40);
    n->sd_lon_m = rd_f32(p + 44);
    n->sd_hgt_m = rd_f32(p + 48);
    n->diff_age_s = rd_f32(p + 56);
    n->n_tracked = p[64];
    n->n_used = p[65];
    n->vel_status = rd_u32(p + 72);
    n->vel_type = rd_u32(p + 76);
    n->hor_speed_mps = rd_f64(p + 88);
    n->track_deg = rd_f64(p + 96);
    n->vert_speed_mps = rd_f64(p + 104);
    n->sd_vert_speed_mps = rd_f32(p + 112);
    n->sd_hor_speed_mps = rd_f32(p + 116);

    /* An epoch with no satellites is not a measurement, whatever its status
     * says (plans/gnss-um98x-config.md, open questions). */
    n->pos_usable = n->sol_status == UM980_SOL_COMPUTED && is_fix_type(n->pos_type) &&
                    n->n_used > 0 && isfinite(n->lat_deg) && isfinite(n->lon_deg) &&
                    isfinite(n->hgt_msl_m) && isfinite(n->undulation_m) &&
                    sigma_ok(n->sd_lat_m) && sigma_ok(n->sd_lon_m) && sigma_ok(n->sd_hgt_m);
    n->vel_usable = n->vel_status == UM980_SOL_COMPUTED && n->vel_type != UM980_TYPE_NONE &&
                    n->n_used > 0 && isfinite(n->hor_speed_mps) && isfinite(n->track_deg) &&
                    isfinite(n->vert_speed_mps) && sigma_ok(n->sd_hor_speed_mps) &&
                    sigma_ok(n->sd_vert_speed_mps);

    time_tag(n);
}

/* Returns true when *out was filled with a BESTNAV epoch. */
static bool handle_frame(um980_t *g, um980_nav_t *out)
{
    uint16_t id = rd_u16(&g->rx[OFS_MSG_ID]);
    uint16_t len = rd_u16(&g->rx[OFS_MSG_LEN]);
    const uint8_t *p = &g->rx[HEADER_LEN];

    g->stats.frames++;

    switch (id) {
    case UM980_MSG_VERSION:
        if (len >= VERSION_MIN_LEN) {
            char sw[33], psn[66];
            memcpy(sw, p + 4, sizeof sw);
            sw[sizeof sw - 1] = '\0';
            memcpy(psn, p + 4 + 33 + 129, sizeof psn);
            psn[sizeof psn - 1] = '\0';
            um980_cfg_on_version(g, rd_u32(p), sw, psn);
        }
        break;

    case UM980_MSG_STADOP:
        if (len >= STADOP_MIN_LEN) {
            g->stats.pdop = rd_f32(p + 8);
            g->stats.hdop = rd_f32(p + 20);
            g->stats.dop_valid = true;
        }
        break;

    case UM980_MSG_BESTNAV:
        if (len >= BESTNAV_MIN_LEN) {
            g->stats.nav_epochs++;
            g->rate_count++;
            if (g->state == UM980_ST_RUN) {
                decode_bestnav(g, p, out);
                return true;
            }
        }
        break;

    default:
        break;
    }
    return false;
}

/* Returns true when *out was filled with a BESTNAV epoch. */
static bool feed(um980_t *g, uint8_t c, um980_nav_t *out)
{
    for (;;) {
        switch (g->rx_mode) {
        case MODE_IDLE:
            if (c == 0xAA) {
                g->rx_mode = MODE_BINARY;
            } else if (c == '$' || c == '#' || c == '<') {
                g->rx_mode = MODE_ASCII;
            } else {
                return false;
            }
            g->rx[0] = c;
            g->rx_len = 1;
            g->rx_need = 0;
            return false;

        case MODE_BINARY:
            if ((g->rx_len == 1 && c != 0x44) || (g->rx_len == 2 && c != 0xB5)) {
                g->rx_mode = MODE_IDLE; /* not a frame: look at c again */
                continue;
            }
            g->rx[g->rx_len++] = c;
            if (g->rx_len == OFS_MSG_LEN + 2u) {
                g->rx_need = HEADER_LEN + rd_u16(&g->rx[OFS_MSG_LEN]) + CRC_LEN;
                if (g->rx_need > UM980_FRAME_MAX) {
                    g->stats.crc_errors++;
                    g->rx_mode = MODE_IDLE;
                }
                return false;
            }
            if (g->rx_need != 0 && g->rx_len == g->rx_need) {
                bool have_nav = false;
                if (crc32(g->rx, g->rx_need - CRC_LEN) == rd_u32(&g->rx[g->rx_need - CRC_LEN])) {
                    have_nav = handle_frame(g, out);
                } else {
                    g->stats.crc_errors++;
                }
                g->rx_mode = MODE_IDLE;
                return have_nav;
            }
            return false;

        case MODE_ASCII:
        default:
            if (c == '\n') {
                g->rx[g->rx_len] = '\0';
                g->stats.lines++;
                g->rx_mode = MODE_IDLE;
                um980_cfg_on_line(g, (const char *)g->rx, g->port->rx_snap_us);
                return false;
            }
            if (c == '\r') {
                return false;
            }
            if (c < 0x20 || c > 0x7E) {
                g->rx_mode = MODE_IDLE; /* not text: look at c again */
                continue;
            }
            if (g->rx_len >= UM980_LINE_MAX - 1u) {
                g->rx_mode = MODE_IDLE;
                return false;
            }
            g->rx[g->rx_len++] = c;
            return false;
        }
    }
}

/* ---- Public ------------------------------------------------------------ */

void um980_init(um980_t *g, uart_dma_t *port, const um980_config_t *cfg)
{
    memset(g, 0, sizeof *g);
    g->cfg = *cfg;
    g->port = port;
    crc_table_init();
    if (cfg->pps_pin >= 0) {
        pps_attach(cfg->pps_pin);
    }
    um980_cfg_init(g, time_us_64());
}

bool um980_send_line(um980_t *g, const char *line)
{
    char b[128];
    int n = snprintf(b, sizeof b, "%s\r\n", line);
    if (n <= 0 || (size_t)n >= sizeof b) {
        return false;
    }
    return uart_dma_write(g->port, b, (size_t)n);
}

bool um980_poll(um980_t *g, um980_nav_t *out)
{
    uart_dma_poll(g->port);
    if (g->port->rx_rd == g->port->rx_wr) {
        uart_dma_rx_snapshot(g->port);
    }

    int c;
    while ((c = uart_dma_getc(g->port)) >= 0) {
        if (feed(g, (uint8_t)c, out)) {
            return true;
        }
    }

    uint64_t now = time_us_64();
    if (now >= g->rate_win_end_us) {
        g->stats.nav_rate_hz = g->rate_count;
        g->rate_count = 0;
        g->rate_win_end_us = now + 1000000u;
    }
    uint64_t t_pps;
    g->stats.pps_edges = pps_read(&t_pps);

    um980_cfg_step(g, now);
    return false;
}

const char *um980_state_name(um980_state_t s)
{
    switch (s) {
    case UM980_ST_DETECT:     return "detect";
    case UM980_ST_CHECK:      return "check";
    case UM980_ST_PROVISION:  return "provision";
    case UM980_ST_PROVE_RATE: return "prove-rate";
    case UM980_ST_RUN:        return "run";
    default:                  return "?";
    }
}

const char *um980_strerror(int err)
{
    switch (err) {
    case UM980_OK:            return "ok";
    case UM980_E_NO_RECEIVER: return "no receiver answered at any baud rate";
    case UM980_E_UNSUPPORTED: return "unsupported receiver or firmware";
    case UM980_E_COMMAND:     return "command rejected or not answered";
    case UM980_E_NO_REBOOT:   return "no answer after a restart";
    case UM980_E_SIGNALGROUP: return "CONFIG does not show SIGNALGROUP 2";
    case UM980_E_NOT_SAVED:   return "configuration check failed after SAVECONFIG and RESET";
    case UM980_E_RATE:        return "BESTNAVB is not at 20 Hz";
    default:                  return "unknown error";
    }
}
