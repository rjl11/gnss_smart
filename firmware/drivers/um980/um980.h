/*
 * Unicore UM980 / UM982 GNSS receiver driver for the Pico SDK.
 *
 *  - Receive: separates ASCII reply lines ($, #, <) from binary frames
 *    (AA 44 B5, CRC-32) on the one port, and decodes BESTNAVB, STADOPB,
 *    UNIHEADINGB (UM982 in dual-antenna mode) and VERSIONB.
 *  - Antenna mode: the caller asks for single or dual. Identify combines that
 *    with the receiver found: dual needs a UM982, otherwise single is used.
 *  - PPS: the GPIO interrupt captures time_us_64() at the rising edge, and
 *    each BESTNAV epoch gets its time of validity on the Pico clock from it.
 *  - Boot state machine (um980_cfg.c): detect the baud rate, identify the
 *    receiver, check its configuration, and provision it if it differs
 *    (plans/gnss-um98x-config.md).
 *
 * The driver uses only COM1 and never sends CONFIG RXTYPE.
 * All writes to the port go through um980_send_line(), the single owner of
 * UART1 TX. RTCM forwarding will go through the same place.
 *
 * Runs on core 0. Not thread safe. One receiver. Error codes are below -100.
 */
#ifndef UM980_H
#define UM980_H

#include <stdbool.h>
#include <stdint.h>

#include "uart_dma.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UM980_OK                 0
#define UM980_E_NO_RECEIVER   (-101) /* nothing answered at any baud rate        */
#define UM980_E_UNSUPPORTED   (-102) /* not a UM980/UM982, or firmware too old   */
#define UM980_E_COMMAND       (-103) /* a command was rejected or not answered   */
#define UM980_E_NO_REBOOT     (-104) /* no answer after FRESET / RESET           */
#define UM980_E_SIGNALGROUP   (-105) /* CONFIG does not show the signal group    */
#define UM980_E_NOT_SAVED     (-106) /* check failed after SAVECONFIG and RESET  */
#define UM980_E_RATE          (-107) /* BESTNAVB is not at 20 Hz                 */

#define UM980_TARGET_BAUD        460800u
#define UM980_PRODUCT_UM982      17
#define UM980_PRODUCT_UM980      18
#define UM980_MIN_BUILD          7923u  /* UM980, MODE ROVER (manual §3.4)    */
#define UM982_MIN_BUILD          7650u  /* UM982, MODE ROVER                  */
#define UM982_SG36_MIN_BUILD     10979u /* UM982 20 Hz on SIGNALGROUP 3 6     */

/* Binary message IDs. */
#define UM980_MSG_VERSION        37
#define UM980_MSG_STADOP         954
#define UM980_MSG_UNIHEADING     972
#define UM980_MSG_BESTNAV        2118

/* Solution status and position/velocity types (manual R1.16, tables 7-175,
 * 7-176). */
#define UM980_SOL_COMPUTED       0
#define UM980_TYPE_NONE          0
#define UM980_TYPE_SINGLE        16
#define UM980_TYPE_PSRDIFF       17
#define UM980_TYPE_SBAS          18
#define UM980_TYPE_L1_FLOAT      32
#define UM980_TYPE_IONOFREE_FLOAT 33
#define UM980_TYPE_NARROW_FLOAT  34
#define UM980_TYPE_L1_INT        48
#define UM980_TYPE_WIDE_INT      49
#define UM980_TYPE_NARROW_INT    50
#define UM980_TYPE_PPP_CONVERGING 68
#define UM980_TYPE_PPP_RTK       71

typedef enum {
    UM980_ST_DETECT = 0,   /* scanning baud rates with VERSIONB             */
    UM980_ST_CHECK,        /* comparing CONFIG / MODE / UNILOGLIST          */
    UM980_ST_PROVISION,    /* FRESET ... SAVECONFIG, RESET                  */
    UM980_ST_PROVE_RATE,   /* after provisioning: BESTNAVB at 20 Hz         */
    UM980_ST_RUN           /* logs flow                                     */
} um980_state_t;

typedef struct {
    int  pps_pin;          /* GPIO wired to PPS, or -1                      */
    bool dual_antenna;     /* antenna-mode setting: true = dual (UM982 only) */
    bool verbose;          /* print state changes and replies with printf   */
} um980_config_t;

/* One UNIHEADING epoch (UM982 in dual-antenna mode, manual R1.16 §7.5.101). */
typedef struct {
    uint64_t t_arrival_us;
    uint64_t t_valid_us;   /* Pico time of validity (PPS), else = arrival   */
    bool     t_from_pps;
    uint16_t week;
    uint32_t tow_ms;

    uint32_t sol_status;   /* UM980_SOL_*                                   */
    uint32_t pos_type;     /* UM980_TYPE_*                                  */
    float    length_m;     /* baseline ANT1 -> ANT2                         */
    float    heading_deg;  /* 0..360, clockwise from true north             */
    float    pitch_deg;
    float    sd_heading_deg;
    float    sd_pitch_deg;
    uint8_t  n_tracked;
    uint8_t  n_used;

    bool     fixed;        /* computed, fixed ambiguities, not INS          */
} um980_heading_t;

/* One BESTNAV epoch. */
typedef struct {
    uint64_t t_arrival_us; /* Pico time when the frame was read             */
    uint64_t t_valid_us;   /* Pico time of validity (PPS), else = arrival   */
    bool     t_from_pps;
    uint16_t week;
    uint32_t tow_ms;
    uint8_t  leap_s;

    uint32_t sol_status;   /* UM980_SOL_*                                   */
    uint32_t pos_type;     /* UM980_TYPE_*                                  */
    double   lat_deg;
    double   lon_deg;
    double   hgt_msl_m;
    float    undulation_m; /* ellipsoid height = hgt_msl_m + undulation_m   */
    float    sd_lat_m, sd_lon_m, sd_hgt_m;
    float    diff_age_s;
    uint8_t  n_tracked;
    uint8_t  n_used;

    uint32_t vel_status;
    uint32_t vel_type;
    double   hor_speed_mps;
    double   track_deg;    /* course over ground, from true north           */
    double   vert_speed_mps; /* positive up                                 */
    float    sd_hor_speed_mps, sd_vert_speed_mps;

    bool     pos_usable;   /* a real fix with satellites and sane sigmas    */
    bool     vel_usable;
} um980_nav_t;

typedef struct {
    uint32_t frames;        /* binary frames with a valid CRC               */
    uint32_t crc_errors;
    uint32_t lines;         /* ASCII lines                                  */
    uint32_t nav_epochs;    /* BESTNAV frames                               */
    uint32_t nav_rate_hz;   /* BESTNAV frames in the last full second       */
    uint32_t hdg_epochs;    /* UNIHEADING frames                            */
    uint32_t pps_edges;
    uint32_t provisions;    /* completed provisioning runs                  */
    float    hdop;          /* from STADOP                                  */
    float    pdop;
    bool     dop_valid;
} um980_stats_t;

#define UM980_FRAME_MAX     1024u
#define UM980_LINE_MAX      512u

typedef struct {
    um980_config_t cfg;
    uart_dma_t    *port;

    /* Receive framer. */
    uint8_t  rx[UM980_FRAME_MAX];
    uint32_t rx_len;
    uint32_t rx_need;      /* binary: total frame length, 0 = unknown       */
    uint8_t  rx_mode;      /* 0 idle, 1 binary, 2 ASCII                     */

    /* Identity (valid once model != 0). */
    int      model;        /* UM980_PRODUCT_*, 0 = not identified           */
    uint32_t build;
    char     sw_version[33];
    char     serial[66];

    /* Effective antenna mode, set at Identify from cfg.dual_antenna and the
     * model. */
    bool     dual;         /* UM982 with both antennas                      */
    bool     dual_refused; /* dual was asked for, but a UM980 is fitted     */
    const char *sg_cmd;    /* "CONFIG SIGNALGROUP ..." for this mode        */

    /* Latest UNIHEADING, handed out once by um980_take_heading(). */
    um980_heading_t hdg;
    bool     hdg_new;

    /* State machine (um980_cfg.c). */
    um980_state_t state;
    int       last_error;
    unsigned  baud;
    unsigned  baud_idx;
    unsigned  step;
    unsigned  retries;
    unsigned  script_pc;
    unsigned  provision_attempts;
    uint64_t  deadline_us;
    uint64_t  step_deadline_us;
    uint64_t  last_line_us;
    bool      proving;       /* in the saved-config proof after provisioning */
    bool      degraded;      /* provisioning failed twice; running as found  */
    bool      got_version;
    int       reply;         /* 0 none, 1 OK, -1 error                       */
    bool      reply_seen_ok;
    uint32_t  cfg_seen;      /* target items found in the CONFIG reply       */
    bool      cfg_bad;       /* an item is present with another value        */
    bool      psrvel_listed;
    bool      mode_ok;
    int       log_count;     /* count line of UNILOGLIST, -1 = not seen      */
    int       log_lines;
    uint32_t  log_seen;
    bool      log_bad;
    uint32_t  win_nav0;

    uint64_t  rate_win_end_us;
    uint32_t  rate_count;

    um980_stats_t stats;
} um980_t;

/* The port must already be initialised (any baud). */
void um980_init(um980_t *g, uart_dma_t *port, const um980_config_t *cfg);

/* Call in a loop from the core 0 main loop until it returns false. Returns
 * true and fills *out for every BESTNAV epoch received in state RUN. When no
 * more data is waiting it runs one step of the state machine and returns
 * false. */
bool um980_poll(um980_t *g, um980_nav_t *out);

/* Returns true once for each UNIHEADING epoch received in state RUN. Call it
 * after um980_poll(). */
bool um980_take_heading(um980_t *g, um980_heading_t *out);

/* Starts detection and provisioning again (the "Reconfigure GNSS" action). */
void um980_reconfigure(um980_t *g);

/* Queues one command line (CR LF is added). The single owner of UART1 TX. */
bool um980_send_line(um980_t *g, const char *line);

static inline um980_state_t um980_state(const um980_t *g) { return g->state; }
static inline const um980_stats_t *um980_stats(const um980_t *g) { return &g->stats; }
static inline int um980_last_error(const um980_t *g) { return g->last_error; }
/* UM980_PRODUCT_UM980 / _UM982, or 0 while not identified. */
static inline int um980_model(const um980_t *g) { return g->model; }
/* Effective antenna mode, valid once um980_model() != 0. */
static inline bool um980_dual(const um980_t *g) { return g->dual; }
/* Dual antenna was asked for, but the receiver is a UM980. */
static inline bool um980_dual_refused(const um980_t *g) { return g->dual_refused; }

const char *um980_state_name(um980_state_t s);
const char *um980_strerror(int err);

/* ---- Internal, shared between um980.c and um980_cfg.c ------------------ */
void um980_cfg_init(um980_t *g, uint64_t now);
void um980_cfg_step(um980_t *g, uint64_t now);
void um980_cfg_on_line(um980_t *g, const char *line, uint64_t now);
void um980_cfg_on_version(um980_t *g, uint32_t product, const char *sw, const char *psn);

#ifdef __cplusplus
}
#endif

#endif /* UM980_H */
