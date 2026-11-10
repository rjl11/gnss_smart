# ESP32-P4 feasibility for INSLIB nav_suite (GNSS 20 Hz, IMU 800 Hz, baro 100 Hz, 10 Hz out)

## Context
This is the same question as [rp2350-feasibility.md](rp2350-feasibility.md), applied to the ESP32-P4.

The ESP32-P4's main ("HP") cores are two RISC-V cores at 360 MHz (400 MHz on rev 3 silicon) with a single-precision FPU, but no double-precision hardware. It has 768 KB of on-chip SRAM, often 16–32 MB of in-package PSRAM, and a 40 MHz low-power core. It has no Wi-Fi or Bluetooth on chip; boards add an ESP32-C6 companion for that. It runs ESP-IDF on FreeRTOS.

## Answer: yes, with more margin than the RP2350 (estimate, not yet measured)
The workload is identical to the RP2350 case: strapdown at 800 Hz, covariance prediction at 20 Hz (`kalman_update_dt_sec` default), GNSS updates at 20 Hz, a 3-state barometer filter at 100 Hz and AHRS. The INSLIB reference figures come from a Cortex-M4F at 180 MHz (STM32F429).

Scaling to a RISC-V core at 360 MHz, with roughly similar work per clock on float code:

| Task | Rate | Est. cost | CPU/s |
|---|---|---|---|
| Strapdown mechanisation | 800 Hz | ~8–12 µs | ~1 % |
| Covariance prediction | 20 Hz | ~0.3–0.5 ms | ~1 % |
| GNSS update (pos+vel) | 20 Hz | ~0.15–0.3 ms | <1 % |
| Baro altitude filter | 100 Hz | <25 µs | <0.3 % |
| AHRS inside nav_suite | 800 Hz / 20 Hz | similar to the INS | ~1–1.5 % |
| Output 10 Hz | 10 Hz | negligible | — |

That totals roughly **2–5 % of one HP core**. Memory isn't a concern: about 13 KB worst-case stack against 768 KB of SRAM.

The one weaker spot is double precision. The few double operations (lat/lon, one coordinate conversion per GNSS fix) run in software, which is slower than the RP2350's double-precision coprocessor. It's still only a few tens of µs per GNSS epoch.

## Real risks (OS, timing and I/O, not compute)
1. **Flash-cache stalls:** while ESP-IDF writes to SPI flash (NVS, logging, OTA), the cache is disabled, and any code or constant data that lives in flash stalls. Put the filter and the sensor interrupt handlers in IRAM (`IRAM_ATTR`, or the `CONFIG_SPI_FLASH_AUTO_SUSPEND` option). Log to an SD card or to PSRAM buffers rather than to internal flash while the filter is running.
2. **Scheduling jitter:** run the filter in a high-priority FreeRTOS task pinned to core 1 and give the drivers, USB and networking core 0. Timestamp samples in the interrupt handler with `esp_timer_get_time()` or a hardware timer capture, not in the task.
3. **TM210 at 800 Hz:** raw data only is 39.2 kB/s. The TM210 has UART only, so use a UART with DMA at 921600 baud ([imu-tm210.md](imu-tm210.md)). The P4 has 5 UARTs plus a low-power UART.
4. **UM980 at 20 Hz:** use Unicore binary logs (BESTNAVB, STADOPB) at 460800 baud ([gnss-um980-config.md](gnss-um980-config.md)), and wire PPS to a GPIO capture to align GNSS and IMU time.
5. **Board-level:** the chip is BGA with more power-supply rails, and it draws noticeably more power than an RP2350 (a few hundred mW vs well under 100 mW). A ready-made dev board is the practical route. Wireless needs the C6 companion.

## Compared with the RP2350
| | RP2350 | ESP32-P4 |
|---|---|---|
| CPU | 2x Cortex-M33, 150 MHz | 2x RISC-V, 360–400 MHz |
| Est. filter load | 5–10 % of one core | 2–5 % of one core |
| Double precision | hardware coprocessor | software |
| Timing determinism | bare metal, very predictable | FreeRTOS plus flash-cache stalls, needs care |
| Power, board design | low power, simple QFN package | higher power, BGA package, needs external radio |
| Fit for this project | ample compute, simplest design | most headroom (e.g. future RTK or logging to SD/USB) |

Both have far more compute than needed. The choice is about I/O, power, connectivity and ecosystem rather than CPU.

## Verification (when implementing)
1. ESP-IDF (master) with `riscv32-esp-elf` is already installed under `C:\esp\master` and `~/.espressif`. Create an IDF project with target `esp32p4` that builds `external/INSLIB/src/*.c` and `KFCore/c/*.c` as a component.
2. Write a bench app that replays an INSLIB dataset at 800/100/20 Hz through `nav_suite_update()`. It should measure cycles per call with `esp_cpu_get_cycle_count()` (worst case and mean) and print the results over USB-serial.
3. Pass criterion: worst case per 1.25 ms IMU tick (with a GNSS update and covariance prediction in the same tick) stays well under 1.25 ms, and average load is below 30 % of one core. Also check that a flash write running at the same time doesn't cause a deadline miss.
4. Check that the output matches a host build of `replay` on the same dataset.
