# ESP32-S3 feasibility for INSLIB nav_suite (GNSS 20 Hz, IMU 800 Hz, baro 100 Hz, 10 Hz out)

## Context
This asks the same question as [rp2350-feasibility.md](rp2350-feasibility.md) and [esp32-p4-feasibility.md](esp32-p4-feasibility.md), this time for the ESP32-S3.

The ESP32-S3 has two Xtensa LX7 cores at 240 MHz with a single-precision FPU. There is no double-precision hardware, and float divide and square root run as multi-instruction sequences. It has 512 KB of SRAM, part of which goes to cache and IRAM. With Wi-Fi, BLE and lwIP running, expect roughly 150–250 KB of free heap. PSRAM is optional, in quad or octal versions.

Wi-Fi 4 (2.4 GHz) and BLE 5 are on the chip itself, but **there is no Bluetooth Classic**. Peripherals:
- 3 UARTs
- 2 general-purpose SPI controllers (SPI2, SPI3)
- 2 I2C controllers
- GDMA
- MCPWM with capture inputs
- USB-Serial-JTAG and USB OTG

There is **no Ethernet MAC**. Firmware runs on ESP-IDF, which is built on FreeRTOS.

The workload is the same as in the other studies:
- TM210 IMU at 800 Hz over UART at 921600 baud, or the ADIS16465 over SPI at 1000 Hz ([imu-adis16465-vs-tm210.md](imu-adis16465-vs-tm210.md))
- UM980 GNSS at 20 Hz with binary logs and PPS
- BMP581 barometer at 100 Hz on I2C
- 10 Hz output over BLE (Nordic UART Service, NUS). W5500 Ethernet is on hold ([ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)).
- a web config page turned on by a GPIO ([web-config-server.md](web-config-server.md))
- RTCM in from SW Maps on Android over the same Bluetooth link, forwarded to UM980 COM2 ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md))

## Answer: yes, there is plenty of compute (estimate, not yet measured)
The scaling starts from INSLIB's reference figures on a Cortex-M4F at 180 MHz (STM32F429). The LX7 at 240 MHz is assumed to do about as much float work per clock as the M4F.

| Task | Rate | Est. cost | CPU/s |
|---|---|---|---|
| Strapdown mechanisation | 800 Hz (1000 Hz with ADIS) | ~12–20 µs | ~1–2 % |
| Covariance prediction | 20 Hz | ~0.4–0.8 ms | ~1–1.5 % |
| GNSS update (pos+vel) | 20 Hz | ~0.2–0.5 ms | <1 % |
| Baro altitude filter | 100 Hz | <40 µs | <0.5 % |
| AHRS inside nav_suite | 800 Hz / 20 Hz | similar to the INS | ~1.5–2.5 % |
| Output 10 Hz | 10 Hz | negligible | — |

That totals roughly **4–8 % of one core**. That's between the RP2350 (5–10 %) and the P4 (2–5 %). As on the P4, the few double operations (lat/lon, one coordinate conversion per fix) run in software and cost a few tens of µs per GNSS epoch. The ~13 KB worst-case stack for `nav_suite_update()` is no problem.

## Real risks and differences from the Pico 2 W design
1. **UARTs: exactly enough, with none spare.** Three ports are needed: the TM210, UM980 COM1, and RTCM out to UM980 COM2. The RTCM comes from SW Maps on the phone over BLE ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)); there is no correction radio. Moving the console to USB-Serial-JTAG frees UART0. There is no PIO, so a fourth port would mean an RMT-based software UART (TX only). Switching to the ADIS16465 on SPI would free a UART.
2. **FreeRTOS can't be avoided.** That goes against "bare metal first" in [rtos-decision.md](rtos-decision.md). The workaround:
   - Pin the filter task to core 1 at the highest priority.
   - Keep drivers, Wi-Fi, BLE, `esp_timer` and IPC tasks on core 0.

   In exchange, NTRIP, the web server and over-the-air updates (OTA) get much simpler. ESP-IDF includes `esp_http_server`, `esp_https_ota` and A/B app partitions.
3. **Flash-cache stalls, same as the P4.** A flash write disables the cache on both cores. The existing plan to pause the filter while saving config ([web-config-server.md](web-config-server.md)) already handles this. Any interrupt handler that must keep running during a write goes in IRAM (`IRAM_ATTR`).
4. **Wi-Fi and BLE interrupt load and RAM.** The Pico 2 W's radio is a separate chip on an SPI link; here it runs on the S3 itself and costs more CPU time and RAM. Keep it on core 0. The bench test must measure filter timing again with web mode and BLE turned on.
5. **Timestamping.** Use MCPWM capture inputs for PPS and the IMU data-ready signal. The hardware latches the timestamp on the APB clock, which is better than reading the time in a GPIO interrupt. Use `esp_timer_get_time()` as the common µs time base.
6. **Ethernet (on hold).** The W5500 on SPI2 is the only option, because the chip has no Ethernet MAC. ESP-IDF has a built-in W5500 driver (`esp_eth`) on lwIP, and WIZnet ioLibrary also works. The AgOpenWeb plan stays the same apart from the pin table and the MAC address, which comes from the factory MAC in eFuse.
7. **Bluetooth.** BLE with NUS on NimBLE works for NMEA out and RTCM in. Bluetooth Classic SPP is not possible, and this now matters: SPP is the fallback if SW Maps' BLE driver doesn't send RTCM ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)). On the S3 there would be no fallback.
8. **Temperature (tractor enclosure).** ESP32-S3-WROOM-1 modules with octal PSRAM (N8R8, N16R8) are rated only to 65 °C. Pick a module without PSRAM, or with quad PSRAM, that is rated to 85 °C, and confirm the rating in its datasheet. This project doesn't need PSRAM.
9. **Porting existing code.** `firmware/drivers/bmp581` is written for the Pico SDK (`hardware/i2c.h`, `time_us_64()`, the GPIO IRQ API). Its glue code needs porting to ESP-IDF's `i2c_master`, GPIO ISR service and `esp_timer`. The Bosch BMP5 API and INSLIB are portable C and need no changes.
10. **Power.** The S3 draws roughly 40–60 mA at 240 MHz with the radio off. Wi-Fi transmit peaks reach about 300 mA, so the regulator has to handle that while web mode is on.

## Compared with the RP2350 and ESP32-P4
| | RP2350 (Pico 2 W) | ESP32-S3 | ESP32-P4 |
|---|---|---|---|
| CPU | 2x Cortex-M33, 150 MHz | 2x Xtensa LX7, 240 MHz | 2x RISC-V, 360–400 MHz |
| Est. filter load | 5–10 % of one core | 4–8 % of one core | 2–5 % of one core |
| Double precision | hardware coprocessor | software | software |
| UARTs | 2 hardware + up to 11 PIO UARTs | 3, none spare | 5 + low-power UART |
| Radio | CYW43439: Wi-Fi + BLE + Classic | on-chip Wi-Fi + BLE | none (needs ESP32-C6) |
| OS / determinism | bare metal possible, very predictable | FreeRTOS required, cache stalls | FreeRTOS required, cache stalls |
| Ethernet (on hold) | W5500 on SPI | W5500 on SPI | built-in EMAC or W5500 |
| NTRIP / OTA / web stack | lwIP by hand, OTA needs work | built into ESP-IDF | built into ESP-IDF |
| Temperature | −20 to 85 °C (Pico 2 W) | 85 °C module without octal PSRAM | depends on board |
| Existing code | BMP581 driver, plans written for it | needs porting | needs porting |

## Bottom line
The ESP32-S3 can do the job, and CPU isn't what decides it. It's stronger on networking software (NTRIP, OTA, web config) and puts the radio on the same chip. It's weaker on spare serial ports and timing determinism, and the existing Pico code would need porting. **Recommendation:** keep the Pico 2 W as the main target. The S3 becomes the better choice in two cases:
- Wi-Fi NTRIP and OTA become core features. Since SW Maps now does NTRIP on the phone, this case is weaker.
- The ADIS16465 (SPI) is adopted, so the UART limit no longer matters.

## Verification (when implementing)
1. ESP-IDF (master) is already installed under `C:\esp\master`. Create a project with target `esp32s3` that builds `external/INSLIB/src/*.c` and `KFCore/c/*.c` as a component.
2. Write a bench app that replays an INSLIB dataset at 800/100/20 Hz through `nav_suite_update()`. It measures cycles per call with `esp_cpu_get_cycle_count()` (worst case and mean) and prints them over USB-Serial-JTAG.
3. Pass criterion: the worst case per 1.25 ms IMU tick (with a GNSS update and a covariance prediction in the same tick) stays well under 1.25 ms, and the average load is below 30 % of one core.
4. Repeat with the Wi-Fi access point and BLE NUS running and a browser polling, and again during an NVS write. Neither may cause a missed deadline.
5. Check that the output matches a host build of `replay` on the same dataset.
