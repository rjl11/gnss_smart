/*
 * SYD Dynamics TransducerM TM210 IMU driver for the Pico SDK.
 *
 *  - EasyProtocol packet codec, following the TM210 layout found on the bench
 *    (plans/imu-tm210-config.md), not the vendor library's.
 *  - Boot state machine: detect the baud rate, check the settings, and if they
 *    differ provision the TM210 (RAM first, flash only after the output has
 *    been verified).
 *  - Raw-data parser (CMD 41): converts to FRD SI units and maps the TM210's
 *    own timestamps onto the Pico microsecond clock (plans/imu-tm210.md).
 *
 * The driver never sends calibration commands (CMD 23).
 *
 * Runs on core 0. Not thread safe. Error codes are below -100.
 */
#ifndef TM210_H
#define TM210_H

#include <stdbool.h>
#include <stdint.h>

#include "uart_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TM210_OK               0
#define TM210_E_NO_IMU      (-101) /* nothing answered at any baud rate          */
#define TM210_E_LAYOUT      (-102) /* Setting object layout not recognised       */
#define TM210_E_NO_ACK      (-103) /* a Setting write was not acknowledged       */
#define TM210_E_READBACK    (-104) /* settings read back differ from the target  */
#define TM210_E_OUTPUT      (-105) /* output is not raw-only at 800 Hz           */
#define TM210_E_LOST        (-106) /* data stopped while running                 */

#define TM210_SETTING_LEN      20
#define TM210_TARGET_BAUD      921600u
#define TM210_TARGET_ODR_HZ    800u

typedef enum {
    TM210_ST_DETECT = 0,     /* scanning baud rates for a Setting reply       */
    TM210_ST_CHECK,          /* settings match, measuring the output rate     */
    TM210_ST_APPLY,          /* writing the target to RAM                     */
    TM210_ST_READBACK,       /* reading the settings back at 921600           */
    TM210_ST_VERIFY_OUTPUT,  /* 2 s: only CMD 41, 800 Hz, no CRC errors       */
    TM210_ST_COMMIT,         /* writing the target to flash                   */
    TM210_ST_COMMIT_READBACK,
    TM210_ST_RUN,            /* samples flow                                  */
    TM210_ST_ERROR           /* layout not recognised: nothing is written     */
} tm210_state_t;

typedef struct {
    bool acc_is_gravity;     /* accelerometer sign, see plans/imu-tm210.md    */
    bool verbose;            /* print state changes with printf               */
} tm210_config_t;

/* One raw sample, already in INSLIB's frame (FRD) and units. */
typedef struct {
    uint64_t t_us;           /* sample time on the Pico clock                 */
    float    dt_s;           /* spacing to the previous sample, TM210 clock   */
    float    gyr[3];         /* rad/s                                         */
    float    acc[3];         /* specific force, m/s^2                         */
    float    mag[3];         /* uT (approximate scale until calibrated)       */
    bool     mag_new;        /* the magnetometer value changed                */
    bool     gap;            /* one or more samples were lost before this one */
} tm210_sample_t;

typedef struct {
    uint32_t packets;        /* valid packets of any type                     */
    uint32_t samples;        /* CMD 41 samples delivered in RUN               */
    uint32_t crc_errors;
    uint32_t gaps;           /* dt above 2.5 ms                               */
    uint32_t time_steps;     /* TM210 timestamp went backwards or jumped      */
    uint32_t provisions;     /* completed provisioning runs                   */
    uint32_t rate_hz;        /* CMD 41 packets in the last full second        */
    float    temperature_c;  /* from the Status object, 1 Hz                  */
    uint16_t internal_rate_hz;
    uint8_t  qos;            /* 0-5                                           */
    bool     status_valid;
} tm210_stats_t;

typedef struct {
    tm210_config_t cfg;
    uart_dma_t    *port;

    /* Packet parser. */
    uint8_t  pkt[80];
    uint32_t pkt_len;        /* bytes collected, including AA 55              */
    uint32_t pkt_need;       /* total bytes of the current packet, 0 = unknown */

    /* State machine. */
    tm210_state_t state;
    int       last_error;
    unsigned  baud;          /* current UART baud                             */
    unsigned  baud_idx;
    unsigned  retries;
    unsigned  step;          /* sub-step inside a state                       */
    uint64_t  deadline_us;
    bool      need_commit;   /* a RAM write happened, flash still differs     */
    bool      got_setting;
    bool      got_ack;
    uint16_t  node_id;
    uint8_t   setting[TM210_SETTING_LEN];
    uint32_t  win_raw;       /* CMD 41 packets in the measuring window        */
    uint32_t  win_other;     /* other output objects in the window            */
    uint32_t  win_crc0;      /* crc_errors at the window start                */

    /* Timestamps. */
    bool      have_time;
    uint32_t  last_ts;       /* last TM210 timestamp (wraps)                  */
    uint64_t  t_imu_us;      /* unwrapped TM210 time                          */
    int64_t   off_cur;       /* min(arrival - t_imu) in the current window    */
    int64_t   off_prev;      /* same, previous window                         */
    uint64_t  off_win_end_us;
    bool      off_prev_valid;
    uint64_t  last_raw_us;   /* arrival of the last CMD 41                    */

    float     last_mag[3];
    uint64_t  next_status_us;
    uint64_t  rate_win_end_us;
    uint32_t  rate_count;

    tm210_stats_t stats;
} tm210_t;

/* The port must already be initialised (any baud). */
void tm210_init(tm210_t *t, uart_dma_t *port, const tm210_config_t *cfg);

/* Call in a loop from the core 0 main loop until it returns false. Returns
 * true and fills *out for every new sample (only in state RUN). When no more
 * data is waiting it runs one step of the state machine and returns false. */
bool tm210_poll(tm210_t *t, tm210_sample_t *out);

/* Starts detection and provisioning again (the "Reconfigure IMU" action). */
void tm210_reconfigure(tm210_t *t);

static inline tm210_state_t tm210_state(const tm210_t *t) { return t->state; }
static inline const tm210_stats_t *tm210_stats(const tm210_t *t) { return &t->stats; }
static inline int tm210_last_error(const tm210_t *t) { return t->last_error; }

const char *tm210_state_name(tm210_state_t s);
const char *tm210_strerror(int err);

#ifdef __cplusplus
}
#endif

#endif /* TM210_H */
