# Web server for configuration and diagnostics (Pico 2 W)

## Context
Host a small web server on the Pico 2 W's on-board CYW43439 Wi-Fi for configuration and diagnostics. **During development it runs all the time**: at boot the Pico joins the open network `TeliaGateway` as a station and serves the pages there. An open access point `gnss_smart` comes later, for the field where no router is around. The production behaviour (Wi-Fi off unless the GPIO22 switch says so, so power stays low and the filter undisturbed) is kept as a later build option.

Bluetooth stays on all the time, because it carries the RTCM corrections from SW Maps ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)), so Wi-Fi and Bluetooth must run together.

Adding Wi-Fi also forces a change to the memory layout: the image can no longer run entirely from SRAM (see "Memory layout").

## When Wi-Fi runs
A cmake option `WEB_MODE` selects it, passed to the code as `APP_WEB_MODE` in `app_config.h`, with the same pattern as `GNSS_ANTENNA_MODE` in `firmware/CMakeLists.txt`:

| `WEB_MODE` | Behaviour |
|---|---|
| `always` (development default) | Wi-Fi and the web server start at boot, after the sensors and core 1 are up, and stay on. |
| `switch` (production, later) | GPIO22 (internal pull-up, switch or jumper to GND, low = web on) is read once at boot. Changing mode means flipping the switch and resetting. The pin is already reserved as `PIN_WEB_MODE_SWITCH` in `firmware/app/board_pins.h`. |
| `off` | No Wi-Fi code linked in. |

- A runtime toggle in `switch` mode (debounce about 50 ms on core 0, start or stop Wi-Fi from the main loop, never from the interrupt handler) stays optional. Bringing up `cyw43_arch` takes a few hundred ms and allocates lwIP buffers, so the boot-time read comes first.
- The on-board LED is driven through the CYW43439 (`CYW43_WL_GPIO_LED_PIN`): slow blink while joining, solid with an IP address, fast blink in AP mode.

## Network modes
### Station on `TeliaGateway` (Phase 2)
- Join `TeliaGateway`, an open network (`CYW43_AUTH_OPEN`). The SSID and password are cmake cache strings `WIFI_STA_SSID` (default `TeliaGateway`) and `WIFI_STA_PASSWORD` (default empty = open), so the network isn't hard-wired in the code. Later the config page can override them from flash.
- **Never block the core 0 loop.** Join with `cyw43_arch_wifi_connect_async()` and check `cyw43_tcpip_link_status()` from the main loop, never the `cyw43_arch_wifi_connect_timeout_ms()` variant: core 0 must keep draining the IMU and GNSS rings (the 8 KB IMU ring holds about 200 ms).
- On a failure (`CYW43_LINK_FAIL`, `NONET`, `BADAUTH`) or a lost link, retry with a backoff (5 s doubling to 30 s). The address comes from lwIP's DHCP client.
- Finding the device:
  - The USB status report prints the Wi-Fi state and IP address every second.
  - DHCP hostname `gnss-smart`, so it shows by name in the router's lease list.
  - lwIP mDNS (`LWIP_MDNS_RESPONDER`) as `gnss-smart.local`. Many Android browsers don't resolve `.local`, so the IP from USB or the router stays the fallback.

### Open access point `gnss_smart` (Phase 3)
- `cyw43_arch_enable_ap_mode("gnss_smart", NULL, CYW43_AUTH_OPEN)`, fixed address 192.168.4.1/24. Open, with no password, by decision (see "Security").
- Needs the small DHCP server from pico-examples (`pico_w/wifi/access_point/dhcpserver`), and optionally its DNS server so any hostname resolves to the device. Copy them into `firmware/drivers/` with their licence.
- When to use it: if the station hasn't joined within about 30 s, or the config page selects AP. The exact rule (fall back and stay, or keep retrying the station in between) is settled when Phase 3 starts.

### Over Ethernet (on hold with the W5500)
The same pages could also be served over the W5500 (ioLibrary has a small HTTP server), so the device can be configured from the tractor network without Wi-Fi. See [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md).

## Software stack (stays bare metal)
- lwIP in `NO_SYS=1` mode via `pico_cyw43_arch_lwip_threadsafe_background`. BTstack later uses the same `cyw43_arch` (CLAUDE.md). This fits the bare-metal layout in [rtos-decision.md](rtos-decision.md): everything network-related runs on core 0, the filter stays alone on core 1.
- A `lwipopts.h` in `firmware/app/`, sized from pico-examples' `lwipopts_examples_common.h`, with `LWIP_MDNS_RESPONDER` and the httpd options added.
- HTTP server: the Pico SDK's `pico_lwip_http` (lwIP httpd). Static pages are compiled into flash with `makefsdata`.
- The web code is its own module (`firmware/app/web/`). `main.c` fills a snapshot struct once a second from the same data `print_status()` prints. The HTTP handlers only read that snapshot and never reach into driver state, so a later move to a FreeRTOS task on core 0 needs no change to the handlers.
- One HTML page with inline JS/CSS (a few tens of KB) polls `/status.json` at 1 Hz. A WebSocket is not needed at first. Config POSTs come in Phase 4.

## Memory layout
The demo firmware is built `copy_to_ram`: the whole image runs from SRAM. Today that is 185 KB of code plus 70 KB of bss out of 520 KB (from `arm-none-eabi-size app.elf`, 2026-11-13). The CYW43439 Wi-Fi firmware alone is about 225 KB, and lwIP, httpd, their buffers and later BTstack come on top (estimate, not yet measured), so that doesn't fit.

Decision: build the **default flash (XIP) binary**, and put only the timing-critical code in SRAM. Done in Phase 1 (2026-11-13), build-checked, not yet run on hardware:
- **How:** `firmware/link/` overrides two Pico SDK linker fragments, `default_text_excludes.incl` and `default_rodata_excludes.incl`, through `pico_add_linker_script_override_path()` in `firmware/CMakeLists.txt`. Every input file excluded there leaves flash and is caught by the `*(.text*)` / `*(.rodata*)` catch-all in the SDK's `.data` section, which crt0 copies to SRAM. No vendor source is changed.
- **What goes to SRAM:** `libinslib.a` and `nav_core1.c` (the core 1 loop and everything under `nav_suite_update()`), the SDK files the loop calls (`queue.c`, `critical_section.c`, `hardware_timer/timer.c`), the SDK's `pico_double` (DCP) and `pico_float` math wrappers that INSLIB's arithmetic and libm calls resolve to, `pico_printf/printf.c` (INSLIB logs through `snprintf` from inside the update), and newlib's `memcpy` / `memset` / `memmove` (they come from `libg.a`). libgcc and libm stay in SRAM as in the SDK default.
- **ISRs:** `pps_irq` (um980) and `bmp581_irq0/1` are `__not_in_flash_func`. The UART receive has no ISR (DMA ring, polled), so nothing to move there.
- **Check:** `python firmware/tools/check_ram_path.py` follows the direct calls in `build/app.dis` from `core1_main` and the ISRs (veneers included, calls through function pointers not) and lists anything reachable in flash. Result: 214 functions reachable, none in flash, the same 214 as the old `copy_to_ram` build.
- **Size (measured, `arm-none-eabi-size -A`):** flash `.text` 60 KB + `.rodata` 7 KB; SRAM `.data` 118 KB (mostly the code above) + `.bss` 68 KB = 186 KB, down from 255 KB with `copy_to_ram`.
- **Still open:** compare core 1's `update_us_max` (already in `nav_output_t`) against the `copy_to_ram` build on the bench. Core 0 code now runs from flash, so its loop is a little slower on a cache miss; watch the IMU/GNSS ring-overflow counters.

This replaces the `copy_to_ram` option in [rp2350-feasibility.md](rp2350-feasibility.md), risk 4.

## Wi-Fi and Bluetooth together
BLE and Wi-Fi can run together on the CYW43439, but they share airtime and RAM. Bluetooth must keep running while Wi-Fi is up, because it carries the RTCM corrections. Start with the default Wi-Fi power-save mode (`cyw43_wifi_pm`) and measure page latency and BLE RTCM throughput when Bluetooth lands. If Bluetooth struggles, lower the NMEA output rate or the page poll rate, but never drop the RTCM input.

## Diagnostics page (read-only)
- Firmware version, build, uptime, reset reason.
- Wi-Fi: mode (station / AP), SSID, link state, IP address, RSSI, reconnect count.
- Per-sensor: measured rate (IMU 800 Hz, baro 100 Hz, GNSS 20 Hz), packets received, checksum/framing errors, UART overruns, time since last sample.
- GNSS: fix type / RTK status, satellites, HDOP, correction age, PPS present and last PPS offset. PPS keeps running for about 30 s after the receiver loses its fix, so "PPS present" doesn't mean a fix. The receiver's PPSSTATUS log (accuracy class, phase error) is a candidate for this line ([gnss-um98x-config.md](gnss-um98x-config.md), "More from the manual (R1.16)").
- GNSS receiver setup: model (UM980 / UM982), effective antenna mode and, when the setting is dual but a UM980 is fitted, a warning; provisioning state (detecting / checking / provisioning step N / running / error), last error, receiver serial number and firmware build ([gnss-um98x-config.md](gnss-um98x-config.md)). From HWSTATUSB: the receiver temperature, labelled "unconfirmed" until the bench check, and the three core supply voltages, each flagged when it is outside its normal range.
- TM210 setup: provisioning state, last error, node ID, and the QoS and temperature from the Status object ([imu-tm210-config.md](imu-tm210-config.md)).
- GNSS heading (dual-antenna mode only): heading solution status and position type, baseline length and its median, heading σ, gated epochs ([gnss-um98x-config.md](gnss-um98x-config.md)).
- Filter: `nav_suite` mode (NONE / ATTITUDE_ONLY / COASTING / FULL, [inslib-nav-suite.md](inslib-nav-suite.md)), position/velocity/attitude, key covariance terms, innovation stats, number of rejected updates.
- Corrections: Bluetooth connected (BLE or SPP), RTCM bytes and frames received from SW Maps, CRC errors, time since the last frame, message types seen ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)).
- Ethernet (on hold): link up/down, `$PAOGI` datagrams sent, RTCM bytes received on UDP 2233.
- CPU: core 1 worst-case and mean time per 1.25 ms IMU tick (DWT cycle counter), core 0 load, free stack/heap.
- Optional: receiver command passthrough (send a command, show the reply) for debugging. It goes through the COM1 TX arbiter between RTCM frames. Block `CONFIG RXTYPE`, so a port can't be locked by mistake.

## Configuration page (write)
- Lever arms (IMU → GNSS antenna), IMU mounting rotation, filter tuning presets.
- GNSS antenna mode: single or dual. The default is the compile-time `APP_GNSS_ANTENNA_MODE`; the saved value overrides it, and "revert to defaults" restores it. It applies after a reboot, not live, and the receiver is re-provisioned on that boot. In dual mode, also the antenna baseline direction `baseline_yaw_b` ([gnss-um98x-config.md](gnss-um98x-config.md), "Antenna mode").
- Output: rate, Bluetooth device name, BLE (NUS) or Classic SPP, message format.
- Ethernet (on hold): IP address, mask, destination address and port (default 192.168.5.124 /24 → 192.168.5.255:9999). The output sentence is always `$PAOGI`. See [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md).
- Sensor setup: the UART baud rates are fixed (460800 for the UM980/UM982, 921600 for the TM210) and not configurable. The receiver configuration is checked at boot and re-provisioned on a mismatch ([gnss-um98x-config.md](gnss-um98x-config.md)). A "Reconfigure GNSS" button forces a full re-provision (`FRESET` and the whole sequence). In the same way, the TM210 is checked at boot ([imu-tm210-config.md](imu-tm210-config.md)), and a "Reconfigure IMU" button forces its provisioning sequence.
- Wi-Fi: station SSID and password (default from the build), station or AP, AP fallback on/off.
- Buttons: save, revert to defaults, reboot.

## Storing configuration
- A versioned struct with a CRC in the last flash sector(s) (4 KB each). Use two sectors alternately so a power cut during a write never loses the config.
- **Flash writes stall execute-in-place on both cores.** Writing needs `flash_safe_execute()`, which pauses core 1 with `multicore_lockout`. With the flash binary (see "Memory layout") this matters more, since core 0 code also runs from flash.
- **Position and attitude output may pause while a setting is applied.** So a save is simple: core 0 tells core 1 to stop the filter, writes flash, then restarts the filter with the new settings. No need to keep the filter running through a flash write.
- Because of that, settings can be applied live without a reboot: the filter re-initialises (re-align, then navigate) with the new lever arms, mounting or tuning. Wi-Fi settings and the GNSS antenna mode still apply on reboot.
- The diagnostics page and the BLE output should show that output is paused (e.g. a status flag in the output message) rather than going silent, so a connected client can tell a config pause from a fault.

## Security
- Development builds join an open network and serve pages without a login. The AP `gnss_smart` is also open, by decision. So anyone in Wi-Fi range can read the diagnostics and, from Phase 4, change the configuration.
- What limits the damage: every POST field is checked against a range before it is stored, and the whole save is rejected if any field is out of range; the passthrough blocks `CONFIG RXTYPE`; the production `switch` build keeps Wi-Fi off unless the switch is set.
- A password for the AP or the pages can be added later without changing the rest of this plan.
- Firmware update over the web (OTA) is out of scope for now. If added, it needs a signed image and the RP2350 A/B partition support.

## Phases
1. **Memory layout:** flash binary with the hot path and ISRs in SRAM, no Wi-Fi yet. Build, then compare core 1 timing on the bench.
2. **Station:** join `TeliaGateway`, status page and `/status.json`, IP on USB, DHCP hostname and mDNS.
3. **Access point:** open `gnss_smart` with the DHCP server, and the fallback rule.
4. **Configuration:** config page, flash storage, "Reconfigure GNSS / IMU" buttons.
5. **Production switch:** `WEB_MODE=switch` with GPIO22.

## Cost (estimate, not yet measured)
- Flash: lwIP + httpd + CYW43 Wi-Fi firmware about 250–350 KB. The Pico 2 W has 4 MB, so it fits.
- SRAM: about 40–80 KB for lwIP buffers and the CYW43 driver, plus the RAM-resident hot path (core 1 loop and INSLIB, a part of today's 185 KB), out of 520 KB.
- CPU: low, all on core 0. Power: Wi-Fi adds roughly 50–100 mA while it is on, which in `always` mode is all the time.

## Verification (when implementing)
1. Phase 1: on the same bench replay ([rp2350-feasibility.md](rp2350-feasibility.md)), core 1's worst case per IMU tick matches the `copy_to_ram` build within noise, and `app.dis` shows no flash calls from the hot path.
2. Phase 2: the Pico joins `TeliaGateway` and prints its IP on USB. A PC or phone on the same network opens `http://<ip>/` (and `gnss-smart.local` where it resolves) and sees values update at 1 Hz. The IMU and GNSS ring-overflow counters stay at 0 while the page polls and while Wi-Fi joins or re-joins (power-cycle the router to test that). Core 1 worst case unchanged with a browser polling.
3. Phase 3: with `TeliaGateway` off, the Pico falls back to the AP. A phone joins `gnss_smart` without a password, gets a 192.168.4.x address and opens 192.168.4.1.
4. Phase 4: save a config, power-cycle, check it's restored. Cut power during a save, check the previous config survives. Change a lever arm live: output pauses with the status flag set, the filter re-aligns and output resumes without a reboot.
5. Phase 5: with `WEB_MODE=switch`, boot with GPIO22 high: Wi-Fi never starts (no SSID visible, current draw unchanged). Boot with it low: Wi-Fi comes up as in Phase 2 or 3.
