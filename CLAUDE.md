# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

GNSS + IMU fusion firmware for an survey and agricultural (AgOpenGPS / AgOpenWeb) navigation unit. `plans/` holds the design. `firmware/` holds a first demo (sensor bring-up and INSLIB fused output, status on USB, no Bluetooth yet). It builds, but as of 2026-11-11 it has not run on hardware. There are no tests yet.

The target system (see `plans/README.md` for the index):
- **MCU:** Raspberry Pi Pico 2 W (RP2350 + CYW43439), Pico SDK, bare metal. FreeRTOS may come later on core 0 only (`plans/rtos-decision.md`).
- **Sensors:** Unicore UM980 or UM982 GNSS on a ready-made module (USB-C = COM3 for a PC, COM1/COM2 UART pins, PPS, 5 V; 20 Hz binary logs on UART1 → COM1, PPS on a GPIO). The model is detected at boot; the antenna mode (single, or dual with GNSS heading on a UM982) is a compile-time default the web page can override, applied after a reboot (`plans/gnss-um98x-config.md`). Also a TM210 IMU (800 Hz raw data only on UART0 at 921600; the MCU configures it at boot, `plans/imu-tm210-config.md`, and never sends TM210 calibration commands), BMP581 barometer (100 Hz on I2C1).
- **Filter:** INSLIB `nav_suite` (`external/INSLIB`). It needs no heap, all its state sits in `nav_suite_t`, and its worst-case stack is about 13 KB (`plans/inslib-nav-suite.md`).
- **Links:** BLE NUS (Classic SPP as fallback) to SW Maps on Android carries NMEA out at 10 Hz and RTCM in. Each CRC-checked RTCM frame goes to the receiver's COM1 on UART1, the same port that carries commands and logs (`plans/rtcm-over-bluetooth.md`). The Pico uses only COM1 (COM3 is the module's USB-C, left for a PC; COM2 goes to the Zero 2 W companion if fitted), and no receiver port is ever locked (`CONFIG RXTYPE` risks bricking it). The MCU configures the receiver itself at boot (`plans/gnss-um98x-config.md`).
- **Wi-Fi web server** for diagnostics and configuration (`plans/web-config-server.md`). During development it is always on and joins the open network `TeliaGateway` as a station, falling back to the open access point `gnss_smart`; a production build (`-DWEB_MODE=switch`) starts Wi-Fi only when GPIO22 is pulled to GND at boot.
- **On hold:** W5500 Ethernet / `$PAOGI` output. Its pins (SPI0, GP16–21) stay reserved, so don't assign them to anything else.

## Core split (applies to all firmware code)

- **Core 1:** only the INSLIB filter, in a plain loop with nothing that preempts it. It pops samples from a core 0 queue and makes one `nav_suite_update()` call per IMU sample (800/s). Every 100 ms it publishes its outputs for core 0 to read.
- **Core 0:** UART DMA, the sensor parsers, the BMP581 driver, Bluetooth (BTstack via `pico_cyw43_arch_threadsafe_background`) and the RTCM forwarder. Write this code as separate modules with queues between them, so a later move to FreeRTOS stays small.
- Take sample timestamps (`time_us_64()`) in the ISR or DMA-complete handler, not when the sample is processed. INSLIB handles measurement delay but needs accurate capture times.
- Put the filter's hot path and the timestamping ISRs in SRAM (`__not_in_flash_func` or RAM section placement) for deterministic timing. A whole-image `copy_to_ram` build won't fit once Wi-Fi is linked in (`plans/web-config-server.md`, "Memory layout").

## Firmware layout and build

- `firmware/CMakeLists.txt` is the project (board `pico2_w`, flash binary with the core 1 path in SRAM through the linker override in `firmware/link/`; stdio on USB only, because UART0 is the TM210). After a build, `python firmware/tools/check_ram_path.py` must report nothing reachable from core 1 or the ISRs in flash; when core 1 starts calling a new SDK or libc function, add its file to `firmware/link/default_text_excludes.incl`.
- `firmware/app/`: `main.c` (core 0 loop and the 1 Hz status report), `nav_core1.c` (the filter on core 1 and its queues), `app_config.h` (mounting, noise figures, options).
- **Every pin, bus instance and baud rate is in `firmware/app/board_pins.h`.** No other file names a GPIO number; drivers get their pins through their config structs.
- `firmware/drivers/`: `uart_dma` (DMA ring receive, polled transmit), `tm210` and `um980` (parser plus boot check/provision state machine each; `um980` drives both the UM980 and the UM982), `bmp581`, `inslib` (CMake wrapper around `external/INSLIB`).

Build on the Windows PC (reload `Path`, `PICO_SDK_PATH`, `picotool_DIR` and `pioasm_DIR` from the User environment first if the shell is stale):

```bash
cmake -S firmware -B firmware/build -G Ninja
ninja -C firmware/build
```

The output is `firmware/build/app.uf2` (and `app.elf` for SWD). The GNSS antenna mode defaults to single; add `-DGNSS_ANTENNA_MODE=dual` to the cmake line for a UM982 with two antennas. The web server is on by default (`-DWEB_MODE=always`, joins `-DWIFI_STA_SSID=TeliaGateway`, open unless `-DWIFI_STA_PASSWORD` is set); `-DWEB_MODE=switch` starts it only when GPIO22 is pulled to GND at boot; `-DWEB_MODE=off` builds without Wi-Fi. `-DWIFI_MODE=auto` (default) falls back to the open AP `gnss_smart` (192.168.4.1) when the station hasn't joined within 30 s of boot and stays there until reboot; `sta` never starts the AP, `ap` runs it from boot. Its code is in `firmware/app/web/` with `firmware/app/lwipopts.h`; the AP's DHCP/DNS servers are vendored in `firmware/drivers/ap_servers/`. On miniPi, follow the rules under "Build host" below.

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

## Build host (DietPi, ROCK 4SE)

A second Linux build host on the LAN: `ssh dietpi@192.168.1.51` (OpenSSH with a key, passwordless sudo). It is a Radxa ROCK 4SE (aarch64, 6 cores, 3.8 GB RAM, no swap, 29 GB SD card) on DietPi / Debian 13. It has none of miniPi's memory limits, so a normal parallel build is fine.

- Repo clone in `~/gnss_smart` (on `main`, pulls from `origin`), Pico SDK 2.3.1 in `~/pico-sdk` (`PICO_SDK_PATH` set in `~/.bashrc`), picotool 2.3.1 in `/usr/local/bin`, Debian's `arm-none-eabi-gcc` 14.2, cmake 3.31, ninja 1.12 (installed 2026-11-13), so the same `cmake -G Ninja` / `ninja` commands as on the PC work.
- Not on Tailscale, so reach it from the LAN only. Its host key changed on 2026-11-13 after a reinstall.

## Build host (miniPi)

Pico SDK builds run on `miniPi`, a Raspberry Pi with only 415 MB of RAM, which you reach as `ssh robert@miniPi` over Tailscale. The SDK is in `~/pico/pico-sdk` (2.3.1), picotool 2.3.1 is in `~/.local/bin`, and `~/pico/pico-examples` is the test project.

- **Set `CMAKE_BUILD_PARALLEL_LEVEL=1` and use `ninja -j1`.** The SDK builds host tools (`pioasm`, and `picotool` when it isn't found) as nested CMake projects with their own `cmake --build`. These ignore the outer `-j1` and run about 5 C++ compiles at once. That pushes the load to 10+, makes the Pi swap until it stops responding, and caused the hard freezes in Nov 2026. The nested builds follow `CMAKE_BUILD_PARALLEL_LEVEL`.
- **A 2 GB swap file (`/var/swap`) is required.** The default zram swap of about 415 MB ran out and froze the Pi. It is set in `/etc/rpi/swap.conf.d/90-build.conf` with `Mechanism=swapfile` and `FixedSizeMiB=2048`. rpi-swap only accepts `swapfile`, `zram`, `zram+file`, `auto` and `none`, and silently treats anything else as `auto`.
- **Use OpenSSH with a key, not Tailscale SSH.** Tailscale SSH is turned off (`tailscale set --ssh=false`) because it uses about 50 MB per connection, against about 10 MB for OpenSSH. A new client needs its key added to `~/.ssh/authorized_keys` on the Pi.
- **Run long builds with `nohup nice -n 10 ... > log`, and watch them over one SSH connection.** Don't reconnect every 30 s. `~/pico/powerlog.sh` logs throttling, temperature, load, memory, swap and the largest processes every 5 s to `~/pico/powerlog.txt`, with an fsync after each line so the log survives a freeze. The journal isn't kept across reboots.

## Git

Work only on `main`. Don't create branches, worktrees or pull requests; commit straight to `main`.

## Plans and docs

- Each plan in `plans/` starts with a **Context** section, cites datasheets by their path in `docs/`, and links related plans by relative link. When adding, renaming or shelving a plan, update the table in `plans/README.md` (shelved plans go under "On hold").
- A hardware or architecture decision usually touches several plans (for example, the pin/UART table in `rp2350-feasibility.md`, the core split in `rtos-decision.md` and the companion plans). Keep them consistent.
- PDFs live in `docs/datasheets/` (component datasheets) and `docs/` (manuals and guides, e.g. the Unicore N4 commands manual). Grep `plans/` for a PDF's path when moving it.
- `docs/firmware/unicore/` holds UM980 / UM982 receiver firmware (`.pkg`, latest build per model) with release notes and English translations in its `README.md`. It is flashed with UPrecise over the module's USB-C (COM3), never from the Pico.
- Open work items are kept in `TODO.md` at the repo root, each linking to the plan that holds the details.
- Figures marked "estimate, not yet measured" in the plans are unverified. Don't present them as measured.
