# RTOS decision: bare metal first, FreeRTOS later

## Recommendation
Start bare metal on the Pico SDK. Move to FreeRTOS when NTRIP over Wi-Fi or SD logging is added.

## Bare-metal layout (start here)
- **Core 1:** only the INSLIB filter, in a plain loop fed by IMU, GNSS and barometer samples from core 0 through a queue (e.g. `pico_util/queue`). Nothing preempts it, so the 800 Hz timing stays deterministic.
- **Core 0:** UART DMA buffers, UM980 and TM210 parsers, BMP581 driver and BLE output. Bluetooth (BTstack) runs in the background via `pico_cyw43_arch_threadsafe_background`, with no RTOS needed.
- INSLIB has no OS dependencies and no heap, so it works either way.

## When FreeRTOS pays off
- **NTRIP client over Wi-Fi:** networking, reconnects and possibly TLS are much easier with blocking sockets in their own task.
- **Several slow jobs at once:** SD-card logging, a config shell, firmware updates or a web UI.

## How to add it later
- Run FreeRTOS on core 0 only (`configNUMBER_OF_CORES = 1`) and keep the filter on bare-metal core 1, so the filter sees no scheduler jitter.
- Give any task that calls `nav_suite_update()` at least about 16 KB of stack (worst case about 13 KB).
- Cost: about 10–20 KB of RAM and a few percent of CPU.
- To keep the move small, write the core 0 code as separate modules with message queues between them from the start.
