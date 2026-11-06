# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

GNSS + IMU fusion firmware for an survey and agricultural (AgOpenGPS / AgOpenWeb) navigation unit. The project is mostly in the planning stage. `plans/` holds the design, and `firmware/` so far has only the BMP581 barometer driver. No top-level firmware project, build or tests exist yet.

The target system (see `plans/README.md` for the index):
- **MCU:** Raspberry Pi Pico 2 W (RP2350 + CYW43439), Pico SDK, bare metal. FreeRTOS may come later on core 0 only (`plans/rtos-decision.md`).
- **Sensors:** Unicore UM980 GNSS (20 Hz binary logs on UART1 → COM1, PPS on a GPIO), TM210 IMU (800 Hz raw data only on UART0 at 921600; the MCU configures it at boot, `plans/imu-tm210-config.md`, and never sends TM210 calibration commands), BMP581 barometer (100 Hz on I2C1).
- **Filter:** INSLIB `nav_suite` (`external/INSLIB`). It needs no heap, all its state sits in `nav_suite_t`, and its worst-case stack is about 13 KB (`plans/inslib-nav-suite.md`).
- **Links:** BLE NUS (Classic SPP as fallback) to SW Maps on Android carries NMEA out at 10 Hz and RTCM in. Each CRC-checked RTCM frame goes to UM980 COM1 on UART1, the same port that carries commands and logs (`plans/rtcm-over-bluetooth.md`). Only COM1 is used, and no UM980 port is ever locked (`CONFIG RXTYPE` risks bricking it). The MCU configures the UM980 itself at boot (`plans/gnss-um980-config.md`).
- **On hold:** W5500 Ethernet / `$PANDA` output. Its pins (SPI0, GP16–21) stay reserved, so don't assign them to anything else.

## Core split (applies to all firmware code)

- **Core 1:** only the INSLIB filter, in a plain loop with nothing that preempts it. It pops samples from a core 0 queue and makes one `nav_suite_update()` call per IMU sample (800/s). Every 100 ms it publishes its outputs for core 0 to read.
- **Core 0:** UART DMA, the sensor parsers, the BMP581 driver, Bluetooth (BTstack via `pico_cyw43_arch_threadsafe_background`) and the RTCM forwarder. Write this code as separate modules with queues between them, so a later move to FreeRTOS stays small.
- Take sample timestamps (`time_us_64()`) in the ISR or DMA-complete handler, not when the sample is processed. INSLIB handles measurement delay but needs accurate capture times.
- Put the filter's hot path in SRAM (`__not_in_flash_func` / `copy_to_ram`) for deterministic timing.

## Drivers

Each driver in `firmware/drivers/<name>/` is a Pico SDK static library with its own `CMakeLists.txt`. The firmware project pulls it in after `pico_sdk_init()` with `add_subdirectory(drivers/<name>)` and `target_link_libraries(app PRIVATE <name>)`. Vendor code is a git submodule in `external/`, compiled into the driver library from its path relative to the driver.

`firmware/drivers/bmp581` is the pattern to follow. It is thin glue around Bosch's API, with I2C timeouts so a stuck bus can't hang core 0, and an INT-pin edge timestamp that is read later by `bmp581_poll()` from the main loop. It is not thread safe and runs on core 0. Driver error codes are below -100, so they don't collide with the vendor's negative codes. It defines `BMP5_USE_FLOATING_POINT` because the RP2350 has an FPU.

## Submodules

```bash
git submodule update --init --recursive
```

- `external/INSLIB`: the navigation filter. It has its own `Makefile`, tests, `coding_style.md` and requirement docs (`requirements/`). Plans cite its headers (`src/nav_suite.h`, `src/ins.h`) as the source of truth.
- `external/BMP5_SensorAPI`: Bosch BMP5 driver.

Not a submodule: `external/syd_dynamics` is a plain vendored copy of SYD Dynamics' TransducerM examples and the TM210 3D model. It is the EasyProfile protocol reference for the TM210 (see its `README.md`), and it is not compiled. `temp/` is git-ignored scratch space.

## Plans and docs

- Each plan in `plans/` starts with a **Context** section, cites datasheets by their path in `docs/`, and links related plans by relative link. When adding, renaming or shelving a plan, update the table in `plans/README.md` (shelved plans go under "On hold").
- A hardware or architecture decision usually touches several plans (for example, the pin/UART table in `rp2350-feasibility.md`, the core split in `rtos-decision.md` and the companion plans). Keep them consistent.
- PDFs live in `docs/datasheets/` (component datasheets) and `docs/` (manuals and guides, e.g. the Unicore N4 commands manual). Grep `plans/` for a PDF's path when moving it.
- Figures marked "estimate, not yet measured" in the plans are unverified. Don't present them as measured.
