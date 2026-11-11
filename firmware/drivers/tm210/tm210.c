/*
 * TM210 driver. Protocol facts and the state machine follow
 * plans/imu-tm210-config.md; data conversion follows plans/imu-tm210.md.
 */
#include "tm210.h"

#include <stdio.h>
#include <string.h>

#include "pico/time.h"

/* EasyProtocol. */
#define EP_SYNC0            0xAA
#define EP_SYNC1            0x55
#define EP_CMD_REQUEST      12
#define EP_CMD_ACK          13
#define EP_CMD_SETTING      21
#define EP_CMD_STATUS       22
#define EP_CMD_RAW          41
#define EP_ID_BROADCAST     0u
#define EP_ID_HOST          2u
#define EP_RAW_LEN          40
#define EP_STATUS_LEN       12

/* Setting object, TM210 layout. */
#define SW_STATUS_OUT       (1u << 5)
#define SW_RAW_OUT          (1u << 6)
#define SW_QUAT_OUT         (1u << 8)
#define SW_EULER_OUT        (1u << 10)
#define SW_RPY_OUT          (1u << 11)
#define SW_GRAVITY_OUT      (1u << 12)
#define SW_RESERVED3        (1u << 13)
#define SW_RESERVED4_MASK   (7u << 14)
#define SW_RESERVED4_VALUE  (2u << 14)
#define SW_REQUEST_ACK      (1u << 17)
#define SW_SAVE_FLASH       (1u << 18)
#define SW_UART_OUT         (1u << 19)
#define SW_USB_OUT          (1u << 20)
#define SW_TEMP_CALIB       (1u << 23)
#define SW_BOOT_MODE_MASK   (3u << 24)
#define SW_CMD42_OUT        (1u << 26)
#define SW_COMBO_OUT        (1u << 27)
#define SW_SLOW_OUT         (1u << 28)

#define SW_MUST_BE_0 (SW_STATUS_OUT | SW_QUAT_OUT | SW_EULER_OUT | SW_RPY_OUT | \
                      SW_GRAVITY_OUT | SW_TEMP_CALIB | SW_BOOT_MODE_MASK |     \
                      SW_CMD42_OUT | SW_COMBO_OUT | SW_SLOW_OUT)
#define SW_MUST_BE_1 (SW_RAW_OUT | SW_UART_OUT | SW_USB_OUT)

#define OFS_SWITCHES        0
#define OFS_BAUD            6   /* uint16, unit 100 bps */
#define OFS_SILENT_TIME     16  /* uint16, seconds      */
#define OFS_ODR             18  /* uint16, Hz           */

/* Timing. */
#define DETECT_WAIT_US          300000u
#define DETECT_WAIT_SLOW_US     1000000u
#define DETECT_RETRY_US         2000000u
#define ACK_WAIT_US             1500000u
#define ACK_RETRIES             5u
#define READBACK_RETRIES        3u
#define BAUD_SWITCH_WAIT_US     300000u
#define CHECK_WINDOW_US         1000000u
#define CHECK_RETRIES           5u
#define VERIFY_WINDOW_US        2000000u
#define LOST_TIMEOUT_US         500000u
#define RATE_TOLERANCE_HZ       20u
#define GAP_US                  2500
#define TIME_STEP_US            1000000
#define OFFSET_WINDOW_US        1000000u
#define PACKET_TX_TIME_US       532 /* 49 bytes at 921600 baud */

/* Conversion (plans/imu-tm210.md). */
#define TM210_G                 9.8158f
#define TM210_MAG_UT            50.0f

static const unsigned k_bauds[] = {
    921600, 115200, 1000000, 460800, 230400, 57600, 38400, 9600, 2400, 1200,
};
#define N_BAUDS (sizeof k_bauds / sizeof k_bauds[0])

#define LOG(t, ...) do { if ((t)->cfg.verbose) { printf("[tm210] " __VA_ARGS__); printf("\n"); } } while (0)

static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static void wr_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static float rd_f32(const uint8_t *p)
{
    float f;
    memcpy(&f, p, sizeof f);
    return f;
}

/* CRC-16 Modbus. */
static uint16_t crc16(const uint8_t *p, size_t n)
{
    uint16_t c = 0xFFFF;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) {
            c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
        }
    }
    return c;
}

static void send_packet(tm210_t *t, uint8_t cmd, uint16_t to_id, const uint8_t *payload, uint8_t plen)
{
    uint8_t b[2 + 1 + 4 + TM210_SETTING_LEN + 2];
    b[0] = EP_SYNC0;
    b[1] = EP_SYNC1;
    b[2] = (uint8_t)(4 + plen);
    wr_u32(&b[3], (uint32_t)cmd | (EP_ID_HOST << 10) | ((uint32_t)to_id << 21));
    memcpy(&b[7], payload, plen);
    wr_u16(&b[7 + plen], crc16(&b[2], 5u + plen));
    uart_dma_write(t->port, b, 9u + plen);
}

static void send_request(tm210_t *t, uint8_t cmd)
{
    const uint8_t payload[4] = { cmd, 0, 0, 0 };
    send_packet(t, EP_CMD_REQUEST, EP_ID_BROADCAST, payload, sizeof payload);
}

/* ---- Settings ---------------------------------------------------------- */

static bool settings_match(const uint8_t *s)
{
    uint32_t sw = rd_u32(&s[OFS_SWITCHES]);
    return (sw & SW_MUST_BE_0) == 0 &&
           (sw & SW_MUST_BE_1) == SW_MUST_BE_1 &&
           rd_u16(&s[OFS_BAUD]) == TM210_TARGET_BAUD / 100u &&
           rd_u16(&s[OFS_SILENT_TIME]) == 0 &&
           rd_u16(&s[OFS_ODR]) == TM210_TARGET_ODR_HZ;
}

/* The values read on our TM210 and on the AiO firmware's TM171. A unit that
 * differs may have another layout, so nothing is written to it. */
static bool layout_recognised(const tm210_t *t)
{
    uint32_t sw = rd_u32(&t->setting[OFS_SWITCHES]);
    return (sw & SW_RESERVED3) != 0 &&
           (sw & SW_RESERVED4_MASK) == SW_RESERVED4_VALUE &&
           rd_u16(&t->setting[OFS_BAUD]) == t->baud / 100u;
}

static void send_target(tm210_t *t, bool save_to_flash)
{
    uint8_t s[TM210_SETTING_LEN];
    memcpy(s, t->setting, sizeof s);

    uint32_t sw = rd_u32(&s[OFS_SWITCHES]);
    sw &= ~(SW_MUST_BE_0 | SW_SAVE_FLASH);
    sw |= SW_MUST_BE_1 | SW_REQUEST_ACK;
    if (save_to_flash) {
        sw |= SW_SAVE_FLASH;
    }
    wr_u32(&s[OFS_SWITCHES], sw);
    wr_u16(&s[OFS_BAUD], (uint16_t)(TM210_TARGET_BAUD / 100u));
    wr_u16(&s[OFS_SILENT_TIME], 0);
    wr_u16(&s[OFS_ODR], (uint16_t)TM210_TARGET_ODR_HZ);

    t->got_ack = false;
    send_packet(t, EP_CMD_SETTING, t->node_id, s, sizeof s);
}

/* ---- Packets ----------------------------------------------------------- */

/* Returns true when t->pkt holds a complete packet with a valid CRC. */
static bool parse_byte(tm210_t *t, uint8_t c)
{
    if (t->pkt_len == 0) {
        if (c == EP_SYNC0) {
            t->pkt[0] = c;
            t->pkt_len = 1;
        }
        return false;
    }
    if (t->pkt_len == 1) {
        if (c == EP_SYNC1) {
            t->pkt[1] = c;
            t->pkt_len = 2;
        } else if (c != EP_SYNC0) {
            t->pkt_len = 0;
        }
        return false;
    }
    if (t->pkt_len == 2) {
        if (c < 4 || (uint32_t)c + 5u > sizeof t->pkt) {
            t->pkt_len = 0;
            return false;
        }
        t->pkt[2] = c;
        t->pkt_len = 3;
        t->pkt_need = (uint32_t)c + 5u;
        return false;
    }
    t->pkt[t->pkt_len++] = c;
    if (t->pkt_len < t->pkt_need) {
        return false;
    }
    if (crc16(&t->pkt[2], t->pkt_need - 4u) != rd_u16(&t->pkt[t->pkt_need - 2u])) {
        t->stats.crc_errors++;
        t->pkt_len = 0;
        return false;
    }
    return true;
}

static bool handle_raw(tm210_t *t, const uint8_t *p, tm210_sample_t *out)
{
    uint64_t arrival = t->port->rx_snap_us;
    uint32_t ts = rd_u32(p);

    t->last_raw_us = arrival;
    if (t->state != TM210_ST_RUN) {
        return false;
    }
    if (!t->have_time) {
        t->have_time = true;
        t->last_ts = ts;
        t->t_imu_us = ts;
        t->off_prev_valid = false;
        t->off_cur = (int64_t)arrival - (int64_t)t->t_imu_us;
        t->off_win_end_us = arrival + OFFSET_WINDOW_US;
        return false; /* the first packet has no dt */
    }

    int32_t delta = (int32_t)(ts - t->last_ts);
    if (delta <= 0 || delta > TIME_STEP_US) {
        t->stats.time_steps++;
        t->have_time = false; /* start again from the next packet */
        return false;
    }
    t->last_ts = ts;
    t->t_imu_us += (uint32_t)delta;

    /* Offset from TM210 time to Pico time: the smallest (arrival - TM210
     * time) over the last one to two seconds is the packet that waited
     * least in the UART and in the main loop. */
    int64_t d = (int64_t)arrival - (int64_t)t->t_imu_us;
    if (arrival >= t->off_win_end_us) {
        t->off_prev = t->off_cur;
        t->off_prev_valid = true;
        t->off_cur = d;
        t->off_win_end_us = arrival + OFFSET_WINDOW_US;
    } else if (d < t->off_cur) {
        t->off_cur = d;
    }
    int64_t off = t->off_cur;
    if (t->off_prev_valid && t->off_prev < off) {
        off = t->off_prev;
    }

    out->t_us = (uint64_t)((int64_t)t->t_imu_us + off - PACKET_TX_TIME_US);
    out->dt_s = (float)delta * 1e-6f;
    out->gap = delta > GAP_US;
    if (out->gap) {
        t->stats.gaps++;
    }

    /* TM210 frame FLU -> INSLIB frame FRD. */
    float gx = rd_f32(p + 4), gy = rd_f32(p + 8), gz = rd_f32(p + 12);
    float ax = rd_f32(p + 16), ay = rd_f32(p + 20), az = rd_f32(p + 24);
    float mx = rd_f32(p + 28), my = rd_f32(p + 32), mz = rd_f32(p + 36);

    out->gyr[0] = gx;
    out->gyr[1] = -gy;
    out->gyr[2] = -gz;
    if (t->cfg.acc_is_gravity) {
        out->acc[0] = -ax * TM210_G;
        out->acc[1] = ay * TM210_G;
        out->acc[2] = az * TM210_G;
    } else {
        out->acc[0] = ax * TM210_G;
        out->acc[1] = -ay * TM210_G;
        out->acc[2] = -az * TM210_G;
    }
    out->mag[0] = mx * TM210_MAG_UT;
    out->mag[1] = -my * TM210_MAG_UT;
    out->mag[2] = -mz * TM210_MAG_UT;

    /* The magnetometer is sampled at 75 Hz and repeated in every packet. */
    out->mag_new = memcmp(out->mag, t->last_mag, sizeof t->last_mag) != 0;
    memcpy(t->last_mag, out->mag, sizeof t->last_mag);

    t->stats.samples++;
    return true;
}

/* Returns true when *out was filled with a new sample. */
static bool handle_packet(tm210_t *t, tm210_sample_t *out)
{
    uint32_t header = rd_u32(&t->pkt[3]);
    uint8_t cmd = (uint8_t)(header & 0x7Fu); /* the 3 bits above CMD are ignored */
    uint16_t from_id = (uint16_t)((header >> 10) & 0x7FFu);
    const uint8_t *payload = &t->pkt[7];
    uint32_t plen = t->pkt[2] - 4u;

    t->stats.packets++;

    switch (cmd) {
    case EP_CMD_RAW:
        if (plen != EP_RAW_LEN) {
            break;
        }
        t->win_raw++;
        t->rate_count++;
        return handle_raw(t, payload, out);

    case EP_CMD_SETTING:
        if (plen == TM210_SETTING_LEN) {
            memcpy(t->setting, payload, TM210_SETTING_LEN);
            t->node_id = from_id;
            t->got_setting = true;
        }
        break;

    case EP_CMD_ACK:
        if (plen >= 1 && payload[0] == EP_CMD_SETTING) {
            t->got_ack = true;
        }
        break;

    case EP_CMD_STATUS:
        if (plen >= EP_STATUS_LEN) {
            t->stats.temperature_c = rd_f32(payload + 4);
            t->stats.internal_rate_hz = rd_u16(payload + 8);
            t->stats.qos = (uint8_t)(rd_u16(payload + 10) & 7u);
            t->stats.status_valid = true;
        }
        if (t->state != TM210_ST_RUN) {
            t->win_other++; /* not requested, so Status output is on */
        }
        break;

    default:
        t->win_other++;
        break;
    }
    return false;
}

/* ---- State machine ----------------------------------------------------- */

static void enter(tm210_t *t, tm210_state_t s)
{
    if (t->state != s) {
        LOG(t, "%s -> %s", tm210_state_name(t->state), tm210_state_name(s));
    }
    t->state = s;
    t->step = 0;
    t->retries = 0;
}

static void enter_detect(tm210_t *t, uint64_t now, uint32_t delay_us)
{
    enter(t, TM210_ST_DETECT);
    t->baud_idx = 0;
    t->deadline_us = now + delay_us;
}

static void fail(tm210_t *t, uint64_t now, int err)
{
    t->last_error = err;
    LOG(t, "error: %s, retrying", tm210_strerror(err));
    enter_detect(t, now, DETECT_RETRY_US);
}

static void start_window(tm210_t *t, uint64_t now, uint32_t len_us)
{
    t->win_raw = 0;
    t->win_other = 0;
    t->win_crc0 = t->stats.crc_errors;
    t->deadline_us = now + len_us;
}

static bool window_ok(const tm210_t *t, uint32_t seconds)
{
    uint32_t want = TM210_TARGET_ODR_HZ * seconds;
    uint32_t tol = RATE_TOLERANCE_HZ * seconds;
    return t->win_raw >= want - tol && t->win_raw <= want + tol &&
           t->win_other == 0 && t->stats.crc_errors == t->win_crc0;
}

static void enter_run(tm210_t *t, uint64_t now)
{
    enter(t, TM210_ST_RUN);
    t->last_error = TM210_OK;
    t->have_time = false;
    t->last_raw_us = now;
    t->next_status_us = now + 1000000u;
}

/* Sends a Setting request and waits for the reply. Returns 1 when the reply
 * is in t->setting, -1 when all retries timed out, 0 while waiting. */
static int readback_step(tm210_t *t, uint64_t now)
{
    if (t->step == 0) {
        t->got_setting = false;
        send_request(t, EP_CMD_SETTING);
        t->deadline_us = now + DETECT_WAIT_US;
        t->step = 1;
        return 0;
    }
    if (t->got_setting) {
        return 1;
    }
    if (now >= t->deadline_us) {
        if (++t->retries >= READBACK_RETRIES) {
            return -1;
        }
        t->step = 0;
    }
    return 0;
}

/* Sends the target and waits for the Ack. Same return values. */
static int write_step(tm210_t *t, uint64_t now, bool save_to_flash)
{
    if (t->step == 0) {
        send_target(t, save_to_flash);
        t->deadline_us = now + ACK_WAIT_US;
        t->step = 1;
        return 0;
    }
    if (t->got_ack) {
        return 1;
    }
    if (now >= t->deadline_us) {
        if (++t->retries >= ACK_RETRIES) {
            return -1;
        }
        t->step = 0;
    }
    return 0;
}

static void step(tm210_t *t, uint64_t now)
{
    int r;

    if (now >= t->rate_win_end_us) {
        t->stats.rate_hz = t->rate_count;
        t->rate_count = 0;
        t->rate_win_end_us = now + 1000000u;
    }

    switch (t->state) {
    case TM210_ST_DETECT:
        if (t->step == 0) { /* pause before a scan */
            if (now >= t->deadline_us) {
                t->step = 1;
            }
        } else if (t->step == 1) { /* try the next baud */
            if (!uart_dma_tx_idle(t->port)) {
                break;
            }
            t->baud = k_bauds[t->baud_idx];
            uart_dma_set_baud(t->port, t->baud);
            t->pkt_len = 0;
            t->got_setting = false;
            send_request(t, EP_CMD_SETTING);
            t->deadline_us = now + (t->baud <= 2400 ? DETECT_WAIT_SLOW_US : DETECT_WAIT_US);
            t->step = 2;
        } else if (t->got_setting) {
            LOG(t, "found at %u baud, node id %u", t->baud, t->node_id);
            if (t->baud == TM210_TARGET_BAUD && settings_match(t->setting)) {
                enter(t, TM210_ST_CHECK);
                start_window(t, now, CHECK_WINDOW_US);
            } else {
                enter(t, TM210_ST_APPLY);
            }
        } else if (now >= t->deadline_us) {
            if (++t->baud_idx >= N_BAUDS) {
                fail(t, now, TM210_E_NO_IMU);
            } else {
                t->step = 1;
            }
        }
        break;

    case TM210_ST_CHECK:
        if (now < t->deadline_us) {
            break;
        }
        if (window_ok(t, 1)) {
            if (t->need_commit) {
                enter(t, TM210_ST_COMMIT);
            } else {
                LOG(t, "settings already match, nothing written");
                enter_run(t, now);
            }
        } else if (++t->retries < CHECK_RETRIES) {
            /* The TM210 answers requests before its output starts (3.2 s
             * cold start), so give it a few windows before provisioning. */
            start_window(t, now, CHECK_WINDOW_US);
        } else {
            LOG(t, "output check failed: %lu raw, %lu other packets in 1 s",
                (unsigned long)t->win_raw, (unsigned long)t->win_other);
            enter(t, TM210_ST_APPLY);
        }
        break;

    case TM210_ST_APPLY:
        if (t->step == 0 && t->retries == 0 && !layout_recognised(t)) {
            t->last_error = TM210_E_LAYOUT;
            if (t->cfg.verbose) {
                printf("[tm210] layout not recognised, nothing written. Setting bytes:");
                for (int i = 0; i < TM210_SETTING_LEN; i++) {
                    printf(" %02x", t->setting[i]);
                }
                printf("\n");
            }
            enter(t, TM210_ST_ERROR);
            break;
        }
        if (t->step <= 1) {
            t->need_commit = true; /* the write may land even if the Ack is lost */
            r = write_step(t, now, false);
            if (r < 0) {
                fail(t, now, TM210_E_NO_ACK);
            } else if (r > 0) {
                LOG(t, "target applied to RAM");
                if (t->baud != TM210_TARGET_BAUD) {
                    t->step = 2;
                } else {
                    enter(t, TM210_ST_READBACK);
                }
            }
        } else if (t->step == 2) { /* switch to the new baud */
            if (uart_dma_tx_idle(t->port)) {
                t->baud = TM210_TARGET_BAUD;
                uart_dma_set_baud(t->port, t->baud);
                t->pkt_len = 0;
                t->deadline_us = now + BAUD_SWITCH_WAIT_US;
                t->step = 3;
            }
        } else if (now >= t->deadline_us) {
            uart_dma_rx_flush(t->port);
            t->pkt_len = 0;
            enter(t, TM210_ST_READBACK);
        }
        break;

    case TM210_ST_READBACK:
        r = readback_step(t, now);
        if (r < 0 || (r > 0 && !settings_match(t->setting))) {
            fail(t, now, TM210_E_READBACK);
        } else if (r > 0) {
            enter(t, TM210_ST_VERIFY_OUTPUT);
            start_window(t, now, VERIFY_WINDOW_US);
        }
        break;

    case TM210_ST_VERIFY_OUTPUT:
        if (now < t->deadline_us) {
            break;
        }
        if (window_ok(t, 2)) {
            enter(t, TM210_ST_COMMIT);
        } else {
            LOG(t, "output check failed: %lu raw, %lu other packets, %lu CRC errors in 2 s",
                (unsigned long)t->win_raw, (unsigned long)t->win_other,
                (unsigned long)(t->stats.crc_errors - t->win_crc0));
            fail(t, now, TM210_E_OUTPUT);
        }
        break;

    case TM210_ST_COMMIT:
        r = write_step(t, now, true);
        if (r < 0) {
            fail(t, now, TM210_E_NO_ACK);
        } else if (r > 0) {
            enter(t, TM210_ST_COMMIT_READBACK);
        }
        break;

    case TM210_ST_COMMIT_READBACK:
        r = readback_step(t, now);
        if (r < 0 || (r > 0 && !settings_match(t->setting))) {
            fail(t, now, TM210_E_READBACK);
        } else if (r > 0) {
            t->need_commit = false;
            t->stats.provisions++;
            LOG(t, "settings saved to flash. Boot mode Auto applies from the next power cycle");
            enter_run(t, now);
        }
        break;

    case TM210_ST_RUN:
        if (now - t->last_raw_us > LOST_TIMEOUT_US) {
            t->last_error = TM210_E_LOST;
            LOG(t, "error: %s", tm210_strerror(TM210_E_LOST));
            enter_detect(t, now, 0);
            break;
        }
        if (now >= t->next_status_us) {
            send_request(t, EP_CMD_STATUS);
            t->next_status_us = now + 1000000u;
        }
        break;

    case TM210_ST_ERROR:
    default:
        break;
    }
}

/* ---- Public ------------------------------------------------------------ */

void tm210_init(tm210_t *t, uart_dma_t *port, const tm210_config_t *cfg)
{
    memset(t, 0, sizeof *t);
    t->cfg = *cfg;
    t->port = port;
    t->state = TM210_ST_DETECT;
    enter_detect(t, time_us_64(), 0);
}

void tm210_reconfigure(tm210_t *t)
{
    t->need_commit = false;
    enter_detect(t, time_us_64(), 0);
}

bool tm210_poll(tm210_t *t, tm210_sample_t *out)
{
    uart_dma_poll(t->port);
    if (t->port->rx_rd == t->port->rx_wr) {
        uart_dma_rx_snapshot(t->port);
    }

    int c;
    while ((c = uart_dma_getc(t->port)) >= 0) {
        if (parse_byte(t, (uint8_t)c)) {
            bool have_sample = handle_packet(t, out);
            t->pkt_len = 0;
            if (have_sample) {
                return true;
            }
        }
    }

    step(t, time_us_64());
    return false;
}

const char *tm210_state_name(tm210_state_t s)
{
    switch (s) {
    case TM210_ST_DETECT:          return "detect";
    case TM210_ST_CHECK:           return "check";
    case TM210_ST_APPLY:           return "apply";
    case TM210_ST_READBACK:        return "readback";
    case TM210_ST_VERIFY_OUTPUT:   return "verify-output";
    case TM210_ST_COMMIT:          return "commit";
    case TM210_ST_COMMIT_READBACK: return "commit-readback";
    case TM210_ST_RUN:             return "run";
    case TM210_ST_ERROR:           return "error";
    default:                       return "?";
    }
}

const char *tm210_strerror(int err)
{
    switch (err) {
    case TM210_OK:         return "ok";
    case TM210_E_NO_IMU:   return "no IMU answered at any baud rate";
    case TM210_E_LAYOUT:   return "Setting layout not recognised";
    case TM210_E_NO_ACK:   return "Setting write not acknowledged";
    case TM210_E_READBACK: return "settings read back differ from the target";
    case TM210_E_OUTPUT:   return "output is not raw-only at 800 Hz";
    case TM210_E_LOST:     return "IMU data stopped";
    default:               return "unknown error";
    }
}
