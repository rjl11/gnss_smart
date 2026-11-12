# Web server for configuration and diagnostics (Pico 2 W, GPIO toggle)

## Context
Host a small web server on the Pico 2 W's on-board CYW43439 Wi-Fi for configuration and diagnostics. It's only on when a GPIO says so, so normal operation keeps Wi-Fi off, the power low and the filter undisturbed. Bluetooth stays on all the time, because it carries the RTCM corrections from SW Maps ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)).

## GPIO toggle
- One free header GPIO (e.g. GPIO22) with the internal pull-up, plus a switch or jumper to GND. Low = web mode on.
- **Read it at boot** (recommended to start): web mode is decided once, so there's no Wi-Fi start or stop while the filter is running. Changing mode means flipping the switch and power-cycling or pressing reset.
- **Runtime toggle** (later, optional): debounce the pin (about 50 ms) on core 0 and start or stop Wi-Fi from the main loop, never from the interrupt handler. Bringing up `cyw43_arch` takes a few hundred ms and allocates lwIP buffers, so it's simpler to keep the boot-time version until it's needed.
- Show the mode on the on-board LED (it's driven through the CYW43439, `CYW43_WL_GPIO_LED_PIN`), e.g. slow blink = web mode.

## Network mode
- **Access point** by default (`cyw43_arch_enable_ap_mode`) with WPA2 and a fixed address (192.168.4.1). This works in the field with no router. It needs the small DHCP server from pico-examples (`pico_w/wifi/access_point/dhcpserver`), plus optionally its DNS server so any hostname resolves to the device.
- **Over Ethernet** (later option, on hold with the W5500): the same pages could also be served over the W5500 (ioLibrary has a small HTTP server), so the device can be configured from the tractor network without Wi-Fi.
- **Station mode** (join an existing network) as an option later. It's the same mode NTRIP over Wi-Fi would need, so it can share that code.
- SSID derived from the board's unique ID (e.g. `gnss-smart-1A2B`). Default password printed on a label or set over USB; changeable from the web page.

## Software stack (stays bare metal)
- lwIP in `NO_SYS=1` mode via `pico_cyw43_arch_lwip_threadsafe_background`. This fits the bare-metal layout in [rtos-decision.md](rtos-decision.md): everything network-related runs on core 0, the filter stays alone on core 1.
- HTTP server: the Pico SDK's `pico_lwip_http` (lwIP httpd). Static pages are compiled into flash with `makefsdata`. Dynamic data comes from a few CGI/POST handlers returning JSON.
- Keep the page small (one HTML file with inline JS/CSS, a few tens of KB). Live values are polled from a JSON endpoint at 1–2 Hz; a WebSocket is not needed at first.
- BLE and Wi-Fi can run together on the CYW43439, but they share airtime and RAM. Bluetooth must keep running in web mode, because it carries the RTCM corrections. Measure it: if it struggles, lower the NMEA output rate while web mode is on, but never drop the RTCM input.
- If FreeRTOS is added later for NTRIP, the web server moves into its own task on core 0 with no change to the handlers.

## Diagnostics page (read-only)
- Firmware version, build, uptime, reset reason.
- Per-sensor: measured rate (IMU 800 Hz, baro 100 Hz, GNSS 20 Hz), packets received, checksum/framing errors, UART overruns, time since last sample.
- GNSS: fix type / RTK status, satellites, HDOP, correction age, PPS present and last PPS offset.
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
- Wi-Fi: SSID, password, AP vs station.
- Buttons: save, revert to defaults, reboot.

## Storing configuration
- A versioned struct with a CRC in the last flash sector(s) (4 KB each). Use two sectors alternately so a power cut during a write never loses the config.
- **Flash writes stall execute-in-place on both cores.** Writing needs `flash_safe_execute()`, which pauses core 1 with `multicore_lockout`.
- **Position and attitude output may pause while a setting is applied.** So a save is simple: core 0 tells core 1 to stop the filter, writes flash, then restarts the filter with the new settings. No need to keep the filter running through a flash write or to run it from SRAM for this reason.
- Because of that, settings can be applied live without a reboot: the filter re-initialises (re-align, then navigate) with the new lever arms, mounting or tuning. Wi-Fi settings and the GNSS antenna mode still apply on reboot.
- The diagnostics page and the BLE output should show that output is paused (e.g. a status flag in the output message) rather than going silent, so a connected client can tell a config pause from a fault.

## Security
- Wi-Fi is off unless the GPIO is set, so the attack surface only exists while someone is deliberately configuring the device.
- WPA2 on the access point; no default open network.
- Validate every POST field against a range before storing it; reject the whole save if any field is out of range.
- Firmware update over the web (OTA) is out of scope for now. If added, it needs a signed image and the RP2350 A/B partition support.

## Cost (estimate, not yet measured)
- Flash: lwIP + httpd + CYW43 Wi-Fi firmware about 250–350 KB. The Pico 2 W has 4 MB, so it fits.
- RAM: about 40–80 KB for lwIP buffers and the CYW43 driver, out of 520 KB.
- CPU: low, all on core 0. Power: Wi-Fi AP adds roughly 50–100 mA while web mode is on.

## Verification (when implementing)
1. Boot with the GPIO high: check that Wi-Fi never starts (no SSID visible, current draw unchanged).
2. Boot with the GPIO low: connect a phone to the AP, open 192.168.4.1, and see live diagnostics update.
3. Run the bench replay from [rp2350-feasibility.md](rp2350-feasibility.md) with web mode on and a browser polling: core 1 worst case per IMU tick must not change measurably.
4. Save a config, power-cycle, check it's restored. Cut power during a save, check the previous config survives.
5. Change a lever arm live: output pauses with the status flag set, the filter re-aligns and output resumes without a reboot.
