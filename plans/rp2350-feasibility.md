# RP2350 feasibility for INSLIB nav_suite (GNSS 20 Hz, IMU 800 Hz, baro 100 Hz, 10 Hz out)

## Context
**Target board: Raspberry Pi Pico 2 W** (RP2350 plus an on-board CYW43439 Wi-Fi/Bluetooth chip). The question is whether an RP2350 (2x Cortex-M33 at 150 MHz, single-precision FPU, double-precision coprocessor, 520 KB SRAM) can run INSLIB's `nav_suite` at these sensor rates.

## Answer: yes, with a large margin (estimate, not yet measured)
What INSLIB says about itself:
- Its reference target is a Cortex-M4F at 180 MHz (STM32F429). The library is designed for this class of MCU: no heap, bounded run time, 32-bit float everywhere except GNSS coordinates (`readme.md`, `requirements/req_sys.md`).
- The strapdown runs at the full IMU rate, but the covariance prediction runs at `kalman_update_dt_sec`, which defaults to 20 Hz (`src/ins.h`). So 800 Hz IMU does **not** mean 800 Kalman predictions per second.
- KFCore benchmark on the M4F: prediction takes about 400–600 µs and update about 100–135 µs (`KFCore/README.md`).
- The worst-case stack for `nav_suite_update()` is about 13 KB (readme, x86_64 figure). That's trivial against 520 KB of SRAM.

Rough CPU budget, scaling the M4F figures to an M33 at 150 MHz:

| Task | Rate | Est. cost | CPU/s |
|---|---|---|---|
| Strapdown mechanisation | 800 Hz | ~15–25 µs | ~2 % |
| Covariance prediction | 20 Hz | ~0.7–1 ms | ~2 % |
| GNSS update (pos+vel) | 20 Hz | ~0.3–0.6 ms | ~1 % |
| Baro altitude filter (3 states) | 100 Hz | <50 µs | <0.5 % |
| AHRS inside nav_suite | 800 Hz / 20 Hz | similar to the INS | ~2–3 % |
| Output 10 Hz | 10 Hz | negligible | — |

That totals roughly **5–10 % of one core**. The second core is free for parsing and I/O. The RP2350's double-precision coprocessor makes the few double operations (lat/lon) cheaper than on the M4F reference.

## Real risks (I/O and timing, not compute)
1. **TM210 at 800 Hz:** this is about 30–50 kB/s depending on packet format. It needs ≥460800 baud UART (or SPI), received via DMA. Check the max output rate and interface in `docs/TransducerM_TM210_Datasheet_EN_V126-R.pdf`.
2. **UM980 at 20 Hz:** use Unicore binary logs (e.g. PVTSLN/BESTNAV) at ≥460800 baud rather than heavy NMEA. Also use PPS so GNSS timestamps line up with IMU time.
3. **Timestamping:** INSLIB tolerates measurement delay through a history buffer (`ins.h`). Every sample still needs an accurate capture timestamp, so take it in the ISR or DMA-complete handler.
4. **Execution from flash:** the RP2350 executes code from external QSPI flash through a cache (XIP). To get deterministic timing, put the filter's hot path in SRAM (`__not_in_flash_func` / `copy_to_ram` build).

## Verification (when implementing)
1. Cross-compile `src/*.c` and `KFCore/c/*.c` with the Pico SDK (arm-none-eabi-gcc, `-O2 -mcpu=cortex-m33 -mfloat-abi=hard`).
2. Write a bench firmware that replays a recorded INSLIB dataset (`datasets/`) at 800/100/20 Hz through `nav_suite_update()`. It should measure per-call cycles with the M33 DWT cycle counter (worst case and mean) and print the results over USB.
3. Pass criterion: worst case per 1.25 ms IMU tick (with a GNSS update and covariance prediction in the same tick) stays well under 1.25 ms, and average load is below 30 % of one core.
4. Check that the output matches a host build of `replay` on the same dataset.

## Serial port layout (2 hardware UARTs is enough)
A second RP2350 linked over SPI (I2C is too slow for streaming) would work. It does mean two firmwares, a framing protocol and a second chip to keep in sync. The same result is available on one chip:

1. **Send RTCM3 straight to the UM980.** The UM980 has three identical UARTs, COM1–COM3 (Unicore N4 manual §4). Wire the radio's TX to UM980 COM2 RX and lock that port with `CONFIG RXTYPE COM2 RTCM`, so it only accepts corrections and ignores commands. No MCU UART is involved. The firmware can still watch correction age and RTK status in the UM980's PVTSLN/BESTNAV output.
2. **Position output goes over the Pico 2 W's on-board Bluetooth**, so it needs no UART. See "Bluetooth output" below.
3. **PIO UARTs are spare capacity.** The RP2350 has 3 PIO blocks with 12 state machines in total. The CYW43439 radio uses one state machine plus a DMA channel for its SPI link (internal pins GPIO23/24/25/29), which leaves 11. The Pico SDK's `pio_uart_tx` / `pio_uart_rx` give one UART per state machine, receiving via DMA, at speeds well beyond 921600 baud. Use them if more serial ports are needed later.

| Port | Device | Direction | Baud |
|---|---|---|---|
| UART0 (hardware) | TM210 IMU | RX (+TX for config) | ≥460800 |
| UART1 (hardware) | UM980 COM1 | RX binary logs, TX config | 460800–921600 |
| On-board CYW43439 (BLE) | Phone / PC | position output 10 Hz | — |
| none (direct wire) | Radio → UM980 COM2 | RTCM3 | radio baud |
| GPIO interrupt | UM980 PPS | time sync | — |
| SPI or I2C | BMP581 | — | — |

Notes:
- The manual says COM1 should be kept for firmware upgrades. With COM1 on the MCU, firmware upgrades of the UM980 go through the RP2350, or use COM3 for the MCU and leave COM1 on a header.
- `CONFIG RXTYPE` can only be issued on COM1.
- If RTCM must go through the MCU (to log it, or to choose between several sources), put the radio on a PIO UART RX and forward the bytes to UM980 COM2 from a PIO TX. That's still one chip.

## Bluetooth output (on-board CYW43439)
- **Profile:** BLE with the Nordic UART Service (NUS), a widely supported "serial over BLE" profile. Apps such as nRF Connect and Serial Bluetooth Terminal read it on iOS and Android. The Pico SDK supports it through BTstack (`pico_btstack_ble`). BTstack also does Bluetooth Classic SPP if Windows/Android serial-port tools are wanted later.
- **Payload:** NMEA GGA/RMC at 10 Hz is about 150 B per frame, so 1.5 kB/s, which fits easily in BLE throughput. Request a larger MTU (e.g. 247) so each sentence fits in one notification.
- **CPU and timing:** run the CYW43 driver and BTstack on core 0 (`pico_cyw43_arch_threadsafe_background` or a FreeRTOS build) and the INSLIB filter on core 1. Bluetooth interrupts then don't disturb the filter's timing. RAM cost is in the tens of KB, a small share of the 520 KB.
- **Antenna:** keep metal and the GNSS antenna away from the board edge that carries the Pico 2 W's antenna. Bluetooth and the GNSS L-band are far apart in frequency, but keep the boards physically separated.
- **Later option:** the Wi-Fi radio could run an NTRIP client, giving RTCM over Wi-Fi or a phone hotspot instead of the external radio. It would forward corrections to UM980 COM2 from a PIO TX, or from UM980 COM3.
