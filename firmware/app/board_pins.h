/*
 * Every pin, bus instance and baud rate of the board, in one place.
 * Nothing else in the firmware names a GPIO number.
 *
 * These are suggested assignments (plans/rp2350-feasibility.md fixes only the
 * peripherals, not the pins). Change them here to match the wiring.
 * Header pin numbers and alternate functions: docs/pico2w-pinout.svg.
 *
 *   GP0   UART0 TX  -> TM210 RXD
 *   GP1   UART0 RX  <- TM210 TXD
 *   GP2   I2C1 SDA  <> BMP581
 *   GP3   I2C1 SCL  -> BMP581
 *   GP4   UART1 TX  -> UM980 COM1 RX
 *   GP5   UART1 RX  <- UM980 COM1 TX
 *   GP6   GPIO in   <- UM980 PPS
 *   GP7   GPIO in   <- BMP581 INT
 *   GP16-21         reserved: SPI0 for the W5500 (on hold)
 *   GP22            reserved: web-mode switch (plans/web-config-server.md)
 *   GP23/24/25/29   used by the CYW43439 on the Pico 2 W
 *
 * The UM980 module runs from 5 V. Check its UART and PPS logic level against
 * the Pico's 3.3 V GPIO before connecting it.
 */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include "hardware/i2c.h"
#include "hardware/uart.h"

/* TM210 IMU on UART0 (plans/imu-tm210.md). */
#define PIN_TM210_UART        uart0
#define PIN_TM210_TX          0
#define PIN_TM210_RX          1
#define TM210_BAUD            921600

/* BMP581 barometer on I2C1 (firmware/drivers/bmp581). */
#define PIN_BMP581_I2C        i2c1
#define PIN_BMP581_SDA        2
#define PIN_BMP581_SCL        3
#define PIN_BMP581_INT        7
#define BMP581_I2C_HZ         400000

/* UM980 COM1 on UART1 (plans/gnss-um98x-config.md). */
#define PIN_UM980_UART        uart1
#define PIN_UM980_TX          4
#define PIN_UM980_RX          5
#define PIN_UM980_PPS         6
#define UM980_BAUD            460800

/* Reserved, not used by this build. Do not assign them to anything else. */
#define PIN_W5500_SPI_FIRST   16
#define PIN_W5500_SPI_LAST    21
#define PIN_WEB_MODE_SWITCH   22

#endif /* BOARD_PINS_H */
