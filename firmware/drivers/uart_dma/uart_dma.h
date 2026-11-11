/*
 * UART port with DMA receive into a ring buffer and a small transmit ring.
 *
 * Receive: one DMA channel in ring mode with an endless transfer count
 * (RP2350 only) copies every byte from the UART into the buffer, so no byte
 * is lost while core 0 is busy. The reader polls the DMA write pointer.
 *
 * Transmit: bytes go into a software ring and uart_dma_poll() moves them to
 * the UART FIFO, so a write never blocks core 0.
 *
 * Runs on core 0. Not thread safe.
 */
#ifndef UART_DMA_H
#define UART_DMA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_DMA_TX_SIZE 512u /* power of two */

typedef struct {
    uart_inst_t *uart;
    int          dma_ch;
    uint8_t     *rx_buf;      /* aligned to its own size                    */
    uint32_t     rx_mask;     /* size - 1                                   */
    uint32_t     rx_rd;       /* read index                                 */
    uint32_t     rx_wr;       /* write index at the last snapshot           */
    uint64_t     rx_snap_us;  /* time_us_64() of the last snapshot          */
    uint32_t     overruns;    /* UART FIFO overruns seen                    */

    uint8_t      tx_buf[UART_DMA_TX_SIZE];
    uint32_t     tx_head;
    uint32_t     tx_tail;
    uint32_t     tx_dropped;  /* bytes refused because the ring was full    */
} uart_dma_t;

/* rx_buf must be 2^rx_size_bits bytes and aligned to that size, for example
 *   static uint8_t buf[4096] __attribute__((aligned(4096)));  -> rx_size_bits 12
 * Returns false if no DMA channel is free. */
bool uart_dma_init(uart_dma_t *u, uart_inst_t *uart, unsigned tx_pin, unsigned rx_pin,
                   unsigned baud, uint8_t *rx_buf, unsigned rx_size_bits);

/* Changes the baud rate and discards everything received so far. Call it only
 * when uart_dma_tx_idle() is true, or the bytes still in the shifter are cut. */
void uart_dma_set_baud(uart_dma_t *u, unsigned baud);

/* Call often from the main loop: moves pending TX bytes into the UART FIFO. */
void uart_dma_poll(uart_dma_t *u);

/* Reads the DMA write pointer and the time together. The bytes returned by
 * uart_dma_getc() up to the next snapshot all arrived before the returned
 * time. */
uint64_t uart_dma_rx_snapshot(uart_dma_t *u);

/* Next byte received before the last snapshot, or -1 if there is none. */
static inline int uart_dma_getc(uart_dma_t *u)
{
    if (u->rx_rd == u->rx_wr) {
        return -1;
    }
    uint8_t c = u->rx_buf[u->rx_rd];
    u->rx_rd = (u->rx_rd + 1u) & u->rx_mask;
    return c;
}

/* Discards all received bytes. */
void uart_dma_rx_flush(uart_dma_t *u);

/* Queues bytes for transmit. Returns false (and sends nothing) if they do not
 * all fit. */
bool uart_dma_write(uart_dma_t *u, const void *data, size_t len);

/* True when the TX ring is empty and the UART has shifted out the last bit. */
bool uart_dma_tx_idle(const uart_dma_t *u);

#ifdef __cplusplus
}
#endif

#endif /* UART_DMA_H */
