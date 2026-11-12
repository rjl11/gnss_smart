#include "uart_dma.h"

#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/time.h"

/* The DMA transfer count, reloaded each time it runs out. A multiple of every
 * ring size, so the count modulo the ring size is the write index. */
#define RX_DMA_COUNT        (1u << 24)

bool uart_dma_init(uart_dma_t *u, uart_inst_t *uart, unsigned tx_pin, unsigned rx_pin,
                   unsigned baud, uint8_t *rx_buf, unsigned rx_size_bits)
{
    u->uart = uart;
    u->rx_buf = rx_buf;
    u->rx_mask = (1u << rx_size_bits) - 1u;
    u->rx_rd = 0;
    u->rx_wr = 0;
    u->rx_snap_us = 0;
    u->overruns = 0;
    u->rx_count = 0;
    u->rx_overflows = 0;
    u->tx_head = 0;
    u->tx_tail = 0;
    u->tx_dropped = 0;

    uart_init(uart, baud);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_hw_flow(uart, false, false);
    uart_set_fifo_enabled(uart, true);
    gpio_set_function(tx_pin, GPIO_FUNC_UART);
    gpio_set_function(rx_pin, GPIO_FUNC_UART);
    gpio_pull_up(rx_pin); /* a floating RX pin must not look like data */

    u->dma_ch = dma_claim_unused_channel(false);
    if (u->dma_ch < 0) {
        return false;
    }

    dma_channel_config c = dma_channel_get_default_config((unsigned)u->dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_ring(&c, true, rx_size_bits); /* wrap the write address */
    channel_config_set_dreq(&c, uart_get_dreq(uart, false));

    dma_channel_configure((unsigned)u->dma_ch, &c, rx_buf, &uart_get_hw(uart)->dr,
                          dma_encode_transfer_count_with_self_trigger(RX_DMA_COUNT), true);
    return true;
}

/* Bytes received so far, modulo RX_DMA_COUNT. The channel reloads its count
 * and goes on when it reaches zero, so the count never stops. */
static inline uint32_t rx_byte_count(const uart_dma_t *u)
{
    uint32_t left = dma_hw->ch[u->dma_ch].transfer_count & DMA_CH0_TRANS_COUNT_COUNT_BITS;
    return (RX_DMA_COUNT - left) & (RX_DMA_COUNT - 1u);
}

uint64_t uart_dma_rx_snapshot(uart_dma_t *u)
{
    uint32_t count = rx_byte_count(u);
    uint32_t unread = ((u->rx_wr - u->rx_rd) & u->rx_mask) +
                      ((count - u->rx_count) & (RX_DMA_COUNT - 1u));
    u->rx_count = count;
    u->rx_wr = count & u->rx_mask;
    if (unread > u->rx_mask) { /* the ring was lapped: what it holds is mixed */
        u->rx_overflows++;
        u->rx_rd = u->rx_wr;
    }
    u->rx_snap_us = time_us_64();

    uart_hw_t *hw = uart_get_hw(u->uart);
    if (hw->rsr & UART_UARTRSR_OE_BITS) {
        u->overruns++;
        hw->rsr = 0; /* any write clears the error flags */
    }
    return u->rx_snap_us;
}

void uart_dma_rx_flush(uart_dma_t *u)
{
    u->rx_count = rx_byte_count(u);
    u->rx_wr = u->rx_count & u->rx_mask;
    u->rx_rd = u->rx_wr;
}

void uart_dma_set_baud(uart_dma_t *u, unsigned baud)
{
    uart_set_baudrate(u->uart, baud);
    uart_get_hw(u->uart)->rsr = 0;
    uart_dma_rx_flush(u);
}

void uart_dma_poll(uart_dma_t *u)
{
    while (u->tx_tail != u->tx_head && uart_is_writable(u->uart)) {
        uart_get_hw(u->uart)->dr = u->tx_buf[u->tx_tail];
        u->tx_tail = (u->tx_tail + 1u) & (UART_DMA_TX_SIZE - 1u);
    }
}

bool uart_dma_write(uart_dma_t *u, const void *data, size_t len)
{
    uint32_t used = (u->tx_head - u->tx_tail) & (UART_DMA_TX_SIZE - 1u);
    if (len > (UART_DMA_TX_SIZE - 1u) - used) {
        u->tx_dropped += (uint32_t)len;
        return false;
    }
    const uint8_t *p = data;
    for (size_t i = 0; i < len; i++) {
        u->tx_buf[u->tx_head] = p[i];
        u->tx_head = (u->tx_head + 1u) & (UART_DMA_TX_SIZE - 1u);
    }
    uart_dma_poll(u);
    return true;
}

bool uart_dma_tx_idle(const uart_dma_t *u)
{
    if (u->tx_tail != u->tx_head) {
        return false;
    }
    return (uart_get_hw(u->uart)->fr & UART_UARTFR_BUSY_BITS) == 0;
}
