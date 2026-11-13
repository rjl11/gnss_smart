/*
 * BMP581 barometer driver for the Raspberry Pi Pico SDK. See bmp581.h.
 */
#include "bmp581.h"

#include <string.h>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/time.h"

/* Longest register write the Bosch API issues over I2C (it writes small
 * config blocks and 2-byte NVM words). */
#define BMP581_MAX_WRITE_LEN 16u

/* I2C timeout: fixed overhead plus generous time per byte at 100 kHz. */
static uint32_t i2c_timeout_us(uint32_t len)
{
    return 1000u + 100u * (len + 2u);
}

/* ------------------------------------------------------------------------ */
/* Bosch API callbacks                                                      */
/* ------------------------------------------------------------------------ */

static BMP5_INTF_RET_TYPE bmp581_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    bmp581_t *b = (bmp581_t *)intf_ptr;
    int r = i2c_write_timeout_us(b->cfg.i2c, b->cfg.addr, &reg_addr, 1, true, i2c_timeout_us(1));
    if (r == 1) {
        r = i2c_read_timeout_us(b->cfg.i2c, b->cfg.addr, data, len, false, i2c_timeout_us(len));
        if (r == (int)len) {
            return BMP5_INTF_RET_SUCCESS;
        }
    }
    b->stats.i2c_errors++;
    return (BMP5_INTF_RET_TYPE)-1;
}

static BMP5_INTF_RET_TYPE bmp581_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    bmp581_t *b = (bmp581_t *)intf_ptr;
    uint8_t buf[1u + BMP581_MAX_WRITE_LEN];

    if (len > BMP581_MAX_WRITE_LEN) {
        return (BMP5_INTF_RET_TYPE)-1;
    }
    buf[0] = reg_addr;
    memcpy(&buf[1], data, len);

    int r = i2c_write_timeout_us(b->cfg.i2c, b->cfg.addr, buf, len + 1u, false, i2c_timeout_us(len + 1u));
    if (r == (int)(len + 1u)) {
        return BMP5_INTF_RET_SUCCESS;
    }
    b->stats.i2c_errors++;
    return (BMP5_INTF_RET_TYPE)-1;
}

static void bmp581_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    sleep_us(period);
}

/* ------------------------------------------------------------------------ */
/* Data-ready interrupt                                                     */
/* ------------------------------------------------------------------------ */

/* The Pico SDK raw GPIO handlers take no argument, so each device slot gets
 * its own small handler. Raw handlers share IO_IRQ_BANK0 with other drivers
 * (W5500 INT, GNSS PPS) instead of taking over the single gpio callback. */
static bmp581_t *s_irq_dev[BMP581_MAX_DEVICES];

/* The handlers run from SRAM, so the INT timestamp doesn't wait on an XIP
 * cache miss. */
static __force_inline void bmp581_irq_common(unsigned slot)
{
    bmp581_t *b = s_irq_dev[slot];
    if (b == NULL) {
        return;
    }
    uint pin = (uint)b->cfg.int_pin;
    if (gpio_get_irq_event_mask(pin) & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE);
        uint64_t now = time_us_64();
        if (b->drdy_pending) {
            b->drdy_missed++;
        }
        b->drdy_t_us = now;
        b->drdy_pending = true;
    }
}

static void __not_in_flash_func(bmp581_irq0)(void) { bmp581_irq_common(0); }
#if BMP581_MAX_DEVICES > 1
static void __not_in_flash_func(bmp581_irq1)(void) { bmp581_irq_common(1); }
#endif
#if BMP581_MAX_DEVICES > 2
#error "Add more bmp581_irqN handlers for BMP581_MAX_DEVICES > 2"
#endif

static irq_handler_t const s_irq_handlers[BMP581_MAX_DEVICES] = {
    bmp581_irq0,
#if BMP581_MAX_DEVICES > 1
    bmp581_irq1,
#endif
};

static int irq_attach(bmp581_t *b)
{
    for (unsigned i = 0; i < BMP581_MAX_DEVICES; i++) {
        if (s_irq_dev[i] == NULL) {
            uint pin = (uint)b->cfg.int_pin;
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_disable_pulls(pin); /* BMP581 INT is push-pull */
            gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_RISE);
            s_irq_dev[i] = b;
            gpio_add_raw_irq_handler(pin, s_irq_handlers[i]);
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE, true);
            irq_set_enabled(IO_IRQ_BANK0, true);
            return BMP581_OK;
        }
    }
    return BMP581_E_NO_IRQ_SLOT;
}

static void irq_detach(bmp581_t *b)
{
    for (unsigned i = 0; i < BMP581_MAX_DEVICES; i++) {
        if (s_irq_dev[i] == b) {
            uint pin = (uint)b->cfg.int_pin;
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE, false);
            gpio_remove_raw_irq_handler(pin, s_irq_handlers[i]);
            s_irq_dev[i] = NULL;
        }
    }
}

/* ------------------------------------------------------------------------ */
/* Public API                                                               */
/* ------------------------------------------------------------------------ */

void bmp581_default_config(bmp581_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->i2c = i2c1;
    cfg->sda_pin = 2;            /* GP2 = I2C1 SDA */
    cfg->scl_pin = 3;            /* GP3 = I2C1 SCL */
    cfg->baud_hz = 400000;
    cfg->addr = BMP5_I2C_ADDR_PRIM;
    cfg->int_pin = -1;

    /* 100 Hz for INSLIB's baro_alt filter. OSR_P 8x / OSR_T 1x fits 100 Hz
     * (the driver checks ODR_IS_VALID). No IIR: INSLIB does the filtering,
     * and the sensor's IIR would add lag that the filter does not model. */
    cfg->odr = BMP5_ODR_100_2_HZ;
    cfg->osr_p = BMP5_OVERSAMPLING_8X;
    cfg->osr_t = BMP5_OVERSAMPLING_1X;
    cfg->iir_p = BMP5_IIR_FILTER_BYPASS;
    cfg->iir_t = BMP5_IIR_FILTER_BYPASS;

    cfg->poll_interval_us = 1000;
}

static int configure_sensor(bmp581_t *b)
{
    int8_t rslt;

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &b->dev);
    if (rslt != BMP5_OK) return rslt;

    rslt = bmp5_get_osr_odr_press_config(&b->osr_odr, &b->dev);
    if (rslt != BMP5_OK) return rslt;

    b->osr_odr.odr = b->cfg.odr;
    b->osr_odr.osr_p = b->cfg.osr_p;
    b->osr_odr.osr_t = b->cfg.osr_t;
    b->osr_odr.press_en = BMP5_ENABLE;
    rslt = bmp5_set_osr_odr_press_config(&b->osr_odr, &b->dev);
    if (rslt != BMP5_OK) return rslt;

    struct bmp5_osr_odr_eff eff;
    rslt = bmp5_get_osr_odr_eff(&eff, &b->dev);
    if (rslt != BMP5_OK) return rslt;
    if (!eff.odr_is_valid) return BMP581_E_ODR_INVALID;

    struct bmp5_iir_config iir = {
        .set_iir_t = b->cfg.iir_t,
        .set_iir_p = b->cfg.iir_p,
        .shdw_set_iir_t = BMP5_ENABLE,  /* data registers carry the filtered value */
        .shdw_set_iir_p = BMP5_ENABLE,
        .iir_flush_forced_en = BMP5_DISABLE,
    };
    rslt = bmp5_set_iir_config(&iir, &b->dev);
    if (rslt != BMP5_OK) return rslt;

    /* Data-ready on INT, pulsed, active high, push-pull. INT_STATUS.drdy is
     * also what poll mode reads, so this is set up with or without the pin. */
    rslt = bmp5_configure_interrupt(BMP5_PULSED, BMP5_ACTIVE_HIGH, BMP5_INTR_PUSH_PULL,
                                    BMP5_INTR_ENABLE, &b->dev);
    if (rslt != BMP5_OK) return rslt;

    struct bmp5_int_source_select src = { .drdy_en = BMP5_ENABLE };
    rslt = bmp5_int_source_select(&src, &b->dev);
    return rslt;
}

int bmp581_init(bmp581_t *b, const bmp581_config_t *cfg)
{
    int rslt;

    if (b == NULL || cfg == NULL || cfg->i2c == NULL ||
        (cfg->addr != BMP5_I2C_ADDR_PRIM && cfg->addr != BMP5_I2C_ADDR_SEC) ||
        ((cfg->sda_pin < 0) != (cfg->scl_pin < 0))) {
        return BMP581_E_BAD_CONFIG;
    }

    memset(b, 0, sizeof(*b));
    b->cfg = *cfg;

    if (cfg->sda_pin >= 0) {
        i2c_init(cfg->i2c, cfg->baud_hz);
        gpio_set_function((uint)cfg->sda_pin, GPIO_FUNC_I2C);
        gpio_set_function((uint)cfg->scl_pin, GPIO_FUNC_I2C);
        /* Weak internal pull-ups only help on a short bench cable. Fit
         * external 2.2-4.7 kOhm pull-ups on the board. */
        gpio_pull_up((uint)cfg->sda_pin);
        gpio_pull_up((uint)cfg->scl_pin);
    }

    b->dev.intf = BMP5_I2C_INTF;
    b->dev.intf_ptr = b;
    b->dev.read = bmp581_i2c_read;
    b->dev.write = bmp581_i2c_write;
    b->dev.delay_us = bmp581_delay_us;

    sleep_ms(2); /* t_startup after power-up */

    rslt = bmp5_soft_reset(&b->dev);
    if (rslt != BMP5_OK) return rslt;

    rslt = bmp5_init(&b->dev);
    if (rslt != BMP5_OK) return rslt;

    rslt = configure_sensor(b);
    if (rslt != BMP581_OK) return rslt;

    if (cfg->int_pin >= 0) {
        rslt = irq_attach(b);
        if (rslt != BMP581_OK) return rslt;
    }

    rslt = bmp5_set_power_mode(BMP5_POWERMODE_NORMAL, &b->dev);
    if (rslt != BMP5_OK) {
        if (cfg->int_pin >= 0) irq_detach(b);
        return rslt;
    }

    b->last_poll_us = time_us_64();
    return BMP581_OK;
}

bool bmp581_poll(bmp581_t *b, bmp581_sample_t *out)
{
    uint64_t t_us;

    if (b->cfg.int_pin >= 0) {
        uint32_t irq = save_and_disable_interrupts();
        bool pending = b->drdy_pending;
        t_us = b->drdy_t_us;
        b->drdy_pending = false;
        b->stats.missed_drdy = b->drdy_missed;
        restore_interrupts(irq);
        if (!pending) {
            return false;
        }
    } else {
        uint64_t now = time_us_64();
        if (now - b->last_poll_us < b->cfg.poll_interval_us) {
            return false;
        }
        b->last_poll_us = now;

        uint8_t status = 0;
        if (bmp5_get_interrupt_status(&status, &b->dev) != BMP5_OK) {
            return false;
        }
        if (!(status & BMP5_INT_ASSERTED_DRDY)) {
            return false;
        }
        t_us = now;
    }

    struct bmp5_sensor_data d;
    if (bmp5_get_sensor_data(&d, &b->osr_odr, &b->dev) != BMP5_OK) {
        return false;
    }

    out->t_us = t_us;
    out->pressure_pa = d.pressure;
    out->temperature_c = d.temperature;
    b->stats.samples++;
    b->stats.last_t_us = t_us;
    return true;
}

void bmp581_deinit(bmp581_t *b)
{
    if (b->cfg.int_pin >= 0) {
        irq_detach(b);
    }
    (void)bmp5_set_power_mode(BMP5_POWERMODE_STANDBY, &b->dev);
}

const char *bmp581_strerror(int err)
{
    switch (err) {
    case BMP581_OK:              return "ok";
    case BMP581_E_BAD_CONFIG:    return "bad config";
    case BMP581_E_ODR_INVALID:   return "ODR too high for OSR settings";
    case BMP581_E_NO_IRQ_SLOT:   return "no free INT slot";
    case BMP5_E_NULL_PTR:        return "null pointer";
    case BMP5_E_COM_FAIL:        return "I2C communication failed";
    case BMP5_E_DEV_NOT_FOUND:   return "device not found";
    case BMP5_E_INVALID_CHIP_ID: return "invalid chip id";
    case BMP5_E_NVM_NOT_READY:   return "NVM not ready";
    case BMP5_E_POR_SOFTRESET:   return "soft reset failed";
    case BMP5_E_INVALID_POWERMODE: return "invalid power mode";
    default:                     return "unknown error";
    }
}
