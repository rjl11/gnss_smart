/*
 * BMP581 barometer driver for the Raspberry Pi Pico SDK (RP2350 / RP2040).
 *
 * Thin glue around Bosch's BMP5 Sensor API (external/BMP5_SensorAPI):
 *  - I2C read/write/delay callbacks on a Pico SDK i2c instance, with timeouts
 *    so a stuck bus never hangs core 0.
 *  - Normal mode at a fixed ODR (default 100 Hz, OSR_P 8x, OSR_T 1x, no IIR).
 *  - Data-ready via the INT pin: the GPIO interrupt captures time_us_64() at
 *    the edge, and bmp581_poll() reads the sample later from the main loop.
 *    Without an INT pin the driver polls INT_STATUS instead (timestamp is then
 *    the read time, so it carries up to one poll interval of jitter).
 *
 * Runs on core 0 (see plans/rtos-decision.md). Not thread safe; one INT pin
 * per device. Up to BMP581_MAX_DEVICES devices with an INT pin.
 */
#ifndef BMP581_H
#define BMP581_H

#include <stdbool.h>
#include <stdint.h>

#include "hardware/i2c.h"
#include "bmp5.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef BMP581_MAX_DEVICES
#define BMP581_MAX_DEVICES 2
#endif

/* bmp581_init() return codes. Negative values below -100 are driver errors;
 * other negative values are BMP5_E_* codes from the Bosch API. */
#define BMP581_OK                 0
#define BMP581_E_BAD_CONFIG    (-101) /* NULL pointer or invalid pin/address  */
#define BMP581_E_ODR_INVALID   (-102) /* ODR too high for the OSR settings    */
#define BMP581_E_NO_IRQ_SLOT   (-103) /* more than BMP581_MAX_DEVICES with INT */

typedef struct {
    i2c_inst_t *i2c;        /* i2c0 or i2c1                                  */
    int         sda_pin;    /* GPIO, or -1 if the bus is set up elsewhere    */
    int         scl_pin;    /* GPIO, or -1 if the bus is set up elsewhere    */
    uint32_t    baud_hz;    /* I2C clock; used only when sda/scl are set     */
    uint8_t     addr;       /* BMP5_I2C_ADDR_PRIM (0x46, SDO=GND), _SEC (0x47),
                             * or 0 to probe 0x46 then 0x47 (init stores the
                             * address that answered in b->cfg.addr) */
    int         int_pin;    /* GPIO wired to the BMP581 INT pin, or -1 to poll */

    uint8_t     odr;        /* BMP5_ODR_*                                    */
    uint8_t     osr_p;      /* BMP5_OVERSAMPLING_*                           */
    uint8_t     osr_t;      /* BMP5_OVERSAMPLING_*                           */
    uint8_t     iir_p;      /* BMP5_IIR_FILTER_*; BYPASS = no added lag      */
    uint8_t     iir_t;      /* BMP5_IIR_FILTER_*                             */

    /* Poll mode only: minimum time between INT_STATUS reads [us]. */
    uint32_t    poll_interval_us;
} bmp581_config_t;

typedef struct {
    uint64_t t_us;          /* Pico time_us_64() at data ready (INT) or read (poll) */
    float    pressure_pa;
    float    temperature_c;
} bmp581_sample_t;

typedef struct {
    uint32_t samples;       /* samples delivered by bmp581_poll()            */
    uint32_t i2c_errors;    /* failed or timed-out I2C transactions          */
    uint32_t missed_drdy;   /* INT edges that arrived before the previous sample was read */
    uint64_t last_t_us;     /* timestamp of the last delivered sample        */
} bmp581_stats_t;

typedef struct {
    bmp581_config_t cfg;
    struct bmp5_dev dev;
    struct bmp5_osr_odr_press_config osr_odr;
    bmp581_stats_t  stats;
    uint64_t        last_poll_us;

    /* Written by the GPIO ISR. */
    volatile bool     drdy_pending;
    volatile uint64_t drdy_t_us;
    volatile uint32_t drdy_missed;
} bmp581_t;

/* Fills in defaults: i2c1 at 400 kHz with no SDA/SCL pins (-1, so the bus is
 * left alone unless the caller sets them from board_pins.h), address probed (0),
 * no INT pin, 100.2 Hz, OSR_P 8x, OSR_T 1x, IIR bypassed, 1 ms poll interval. */
void bmp581_default_config(bmp581_config_t *cfg);

/* Sets up the bus (if pins are given), soft-resets and configures the sensor,
 * attaches the INT handler, and starts normal mode.
 * Returns BMP581_OK or a negative error (BMP581_E_* or BMP5_E_*). */
int bmp581_init(bmp581_t *b, const bmp581_config_t *cfg);

/* Call often from the core 0 main loop. Returns true and fills *out when a new
 * sample was read. Never blocks for longer than one short I2C transaction. */
bool bmp581_poll(bmp581_t *b, bmp581_sample_t *out);

/* Puts the sensor into standby and detaches the INT handler. */
void bmp581_deinit(bmp581_t *b);

static inline const bmp581_stats_t *bmp581_stats(const bmp581_t *b) { return &b->stats; }

/* Human-readable name for a bmp581_init() return code. */
const char *bmp581_strerror(int err);

#ifdef __cplusplus
}
#endif

#endif /* BMP581_H */
