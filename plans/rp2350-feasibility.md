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
1. **TM210 at 800 Hz:** raw data only is 49 B per packet, so 39.2 kB/s. Run UART at 921600 baud (about 43 % load, the vendor's recommendation for ODR ≥400 Hz), received via DMA. Configuration and conversion: [imu-tm210.md](imu-tm210.md). How samples go into `nav_suite`: [inslib-nav-suite.md](inslib-nav-suite.md).
2. **UM980 at 20 Hz:** use Unicore binary logs (BESTNAVB at 20 Hz, STADOPB at 1 Hz) at 460800 baud rather than heavy NMEA. Also use PPS so GNSS timestamps line up with IMU time. The MCU configures the receiver itself at boot: [gnss-um980-config.md](gnss-um980-config.md). The UM982 dual-antenna variant uses the same UART1/COM1 layout and adds UNIHEADINGB at 20 Hz (about 1.4 kB/s): [gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md).
3. **Timestamping:** INSLIB tolerates measurement delay through a history buffer (`ins.h`). Every sample still needs an accurate capture timestamp, so take it in the ISR or DMA-complete handler.
4. **Execution from flash:** the RP2350 executes code from external QSPI flash through a cache (XIP). To get deterministic timing, put the filter's hot path in SRAM (`__not_in_flash_func` / `copy_to_ram` build).

## Verification (when implementing)
1. Cross-compile `src/*.c` and `KFCore/c/*.c` with the Pico SDK (arm-none-eabi-gcc, `-O2 -mcpu=cortex-m33 -mfloat-abi=hard`).
2. Write a bench firmware that replays a recorded INSLIB dataset (`datasets/`) at 800/100/20 Hz through `nav_suite_update()`. It should measure per-call cycles with the M33 DWT cycle counter (worst case and mean) and print the results over USB.
3. Pass criterion: worst case per 1.25 ms IMU tick (with a GNSS update and covariance prediction in the same tick) stays well under 1.25 ms, and average load is below 30 % of one core.
4. Check that the output matches a host build of `replay` on the same dataset.

## Serial port layout (2 hardware UARTs is enough)
A second RP2350 linked over SPI (I2C is too slow for streaming) would work. It does mean two firmwares, a framing protocol and a second chip to keep in sync. The same result is available on one chip:

1. **RTCM3 comes from the phone over Bluetooth and goes into UM980 COM1, the same port as commands and logs.**
   - The UM980 has three identical UARTs, COM1–COM3 (Unicore N4 manual §4). It sits on a ready-made module whose USB-C port is COM3, with COM1 and COM2 on pins. The Pico uses only COM1. COM3 (USB-C) is left for a PC, and the COM2 pins are not connected. All are set to 460800 and left unlocked.
   - SW Maps on Android runs the NTRIP client and sends RTCM over the same Bluetooth link that carries our NMEA. There is no correction radio.
   - The Pico checks each RTCM frame and writes it to UART1 TX through the `um980_tx` arbiter. The arbiter writes only whole frames and whole command lines, so they never interleave ([gnss-um980-config.md](gnss-um980-config.md)).
   - **No port is locked** (`CONFIG RXTYPE` is never sent, because of the risk of bricking). COM1 stays at `AUTO` and recognises RTCM by itself.
   - The firmware watches correction age and RTK status in the UM980's BESTNAV output.
   - Details: [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md).
2. **Position output goes over the Pico 2 W's on-board Bluetooth**, so it needs no UART. See "Bluetooth output" below. A W5500 Ethernet module on SPI (UDP `$PANDA` for AgOpenWeb) is **on hold**: its plan is kept in [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md), and its pins stay reserved.
3. **PIO UARTs are spare capacity.** The RP2350 has 3 PIO blocks with 12 state machines in total. The CYW43439 radio uses one state machine plus a DMA channel for its SPI link (internal pins GPIO23/24/25/29), which leaves 11. The Pico SDK's `pio_uart_tx` / `pio_uart_rx` give one UART per state machine, receiving via DMA, at speeds well beyond 921600 baud. Use them if more serial ports are needed later.

| Port | Device | Direction | Baud |
|---|---|---|---|
| UART0 (hardware) | TM210 IMU | RX raw data, TX config + status request | 921600 |
| UART1 (hardware) | UM980 COM1 | RX binary logs, TX config + RTCM | 460800 |
| On-board CYW43439 (BLE, or Classic SPP as fallback) | Phone (SW Maps) | position output 10 Hz, RTCM in | — |
| SPI0 (GP16–21) | W5500 Ethernet (on hold, pins reserved) | UDP `$PANDA` out 10 Hz, RTCM in (also to COM1) | ~30 MHz SPI |
| GPIO interrupt | UM980 PPS | time sync | — |
| I2C1 (e.g. GP2/GP3) | BMP581 | — | — |

Notes:
- **GNSS module:** the UM980 (and the UM982 variant) is a ready-made module with USB-C (COM3), COM1 RX/TX and COM2 RX/TX on pins, PPS, 5 V and GND. It takes 5 V, so it runs from the 5 V rail, not the Pico's 3V3. Check the module's UART and PPS logic level against the Pico's 3.3 V GPIO on the bench.
- **Receiver firmware upgrades are out of scope for this project.** They are done with UPrecise on a Windows PC over the module's USB-C (COM3), so the firmware needs no passthrough for them.
- UM980 COM2 is not connected to the MCU. It runs at 460800 and is unlocked, so a PC tool on its pins can use it.
- With the Zero 2 W companion, COM2 and PPS also go to the Zero, and RTCM enters COM2 from the Zero instead of COM1 through the Pico ([pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md)).
- All RTCM now passes through the MCU (Bluetooth → UART1 TX → COM1), so it can be logged or checked for free.

## Bluetooth link: output and RTCM in (on-board CYW43439)
- **Client:** SW Maps on Android connects as to a Bluetooth GNSS receiver. It reads our NMEA and sends RTCM from its NTRIP client back over the same link. Our GGA must carry a real position as soon as the UM980 has a fix, because SW Maps forwards it to the caster. See [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md).
- **Profile:** BLE with the Nordic UART Service (NUS), a widely supported "serial over BLE" profile. The Pico SDK supports it through BTstack (`pico_btstack_ble`). If SW Maps' BLE driver doesn't send RTCM to NUS, fall back to Bluetooth Classic SPP, which BTstack also supports on the CYW43439.
- **Payload:** NMEA GGA/RMC at 10 Hz is about 150 B per frame, so 1.5 kB/s, which fits easily in BLE throughput. Request a larger MTU (e.g. 247) so each sentence fits in one notification.
- **CPU and timing:** run the CYW43 driver and BTstack on core 0 (`pico_cyw43_arch_threadsafe_background` or a FreeRTOS build) and the INSLIB filter on core 1. Bluetooth interrupts then don't disturb the filter's timing. RAM cost is in the tens of KB, a small share of the 520 KB.
- **Antenna:** keep metal and the GNSS antenna away from the board edge that carries the Pico 2 W's antenna. Bluetooth and the GNSS L-band are far apart in frequency, but keep the boards physically separated.
- **Later option (not needed for now):** an NTRIP client on the Pico's own Wi-Fi, using a phone hotspot. This would remove the need to keep SW Maps running. It would feed the same UART1 TX → COM1 path.
