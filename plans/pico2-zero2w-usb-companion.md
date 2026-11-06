# Raspberry Pi Zero 2 W as Linux companion to a plain Pico 2 over USB

## Context
Can a Raspberry Pi Zero 2 W run this project? It has plenty of CPU: four Cortex-A53 cores at 1 GHz with hardware double precision, and 512 MB of RAM. It also has 2.4 GHz Wi-Fi, Bluetooth 4.2 (BLE and Classic) and one micro-USB OTG port. On its own it is still a poor fit:
- **Not enough UARTs.** Only one full PL011 UART and a mini UART reach the header, and onboard Bluetooth normally uses the PL011. The TM210 at 921600 and UM980 COM1 can't both get a usable hardware UART.
- **Timestamp jitter.** User space sees UART bytes only after tty buffering, so the timing jitter is in milliseconds. The design needs ISR-level capture times ([rp2350-feasibility.md](rp2350-feasibility.md)).
- **Slow boot and fragile storage.** Linux takes tens of seconds to boot, and the SD card is at risk when power is cut.

This plan asks the same question as [esp32-s3-wifi-companion.md](esp32-s3-wifi-companion.md), but with a Zero 2 W as the companion. A plain Pico 2 keeps the sensors and INSLIB, and the Zero joins over USB as a Linux companion. Two extra wires go straight from the UM980 to the Zero: COM2, and the PPS signal for a stratum 1 NTP server. The Zero also drives the Pico's SWD port, so the first hardware build doubles as the debug rig.

No Zero 2 W product brief is in `docs/datasheets/` yet. Board facts here come from Raspberry Pi's public specs and should be checked against the brief once it is added.

## Answer: yes, with each board owning whole features
- **The Pico owns the sensors, timing and filter:** the TM210, UM980 COM1, the BMP581, PPS capture and INSLIB.
- **The Zero owns networking, storage, time service and debugging:** the Bluetooth link to SW Maps, NTRIP over Wi-Fi, RTCM into the UM980, web config, raw logging, NTP, SWD and firmware updates.
- **The USB link to the Pico needs no UART** and plays no part in the timing. Every capture timestamp is still taken on the Pico.

## Recommended split
- **Plain Pico 2 (no W):**
  - TM210 on UART0, UM980 COM1 on UART1, BMP581 on I2C1, PPS on a GPIO interrupt. SPI0 (GP16–21) stays reserved for the W5500, which is on hold ([ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)).
  - Core 1 runs only the filter, as in [rtos-decision.md](rtos-decision.md).
  - Core 0 runs UART DMA, the parsers, the BMP581 driver, the `um980_tx` arbiter (commands only, see below) and TinyUSB. TinyUSB replaces BTstack and the CYW43 driver, which means less code and fewer interrupts on core 0.
  - The Pico still configures the UM980 itself at boot ([gnss-um980-config.md](gnss-um980-config.md)), and it is the only device that sends it commands.
- **Zero 2 W (Raspberry Pi OS Lite, read-only root):** one daemon, plus gpsd, chrony and OpenOCD, owns:
  - The Bluetooth link to SW Maps through BlueZ: BLE NUS, with Classic SPP as the fallback. Unlike with the ESP32-S3, the SPP fallback survives.
  - An NTRIP client over Wi-Fi, so RTK works without SW Maps.
  - Optionally, AgOpenGPS/AgIO UDP over Wi-Fi, with the payload defined in the on-hold Ethernet plan. The W5500 stays on hold.
  - The web config and diagnostics page, which replaces [web-config-server.md](web-config-server.md) on the Pico.
  - Raw sensor logging for replay through INSLIB.
  - A stratum 1 NTP server (see below).
  - SWD flashing and debugging of the Pico, plus `picotool` updates in the field.
- **Why a plain Pico 2:** the Zero has both BLE and Classic Bluetooth, so the CYW43439 adds nothing. Keep the Pico 2 W only if phone output must work during the Zero's boot or after a Zero failure (see "Power, boot and failure").

### RTCM path: straight into COM2, not through the Pico
- NTRIP over Wi-Fi, or SW Maps over BLE/SPP, arrives at the Zero. From there it goes out the Zero's UART TX (GPIO14) into **UM980 COM2 RX**.
- **No port is locked.** `CONFIG RXTYPE` is never sent, because of the risk of bricking. COM2 stays at the default `AUTO` and recognises RTCM by itself, as COM1 does in [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md).
- **The Zero writes only RTCM:** whole RTCM3 frames that pass CRC-24Q. It never writes command lines, so the receiver has a single configuring device, the Pico on COM1.
- **One correction source at a time.** The daemon chooses between NTRIP and SW Maps.
- **On the Pico,** the `um980_tx` arbiter on COM1 now carries only commands. The Pico still watches correction age and RTK status in BESTNAVB.
- **Corrections keep flowing** while the Pico reboots, updates its firmware or is halted by a debugger.
- **Without the Zero** (a Pico 2 W build), RTCM goes back to COM1 through the Pico, as in [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md).

## USB link (Pico ↔ Zero)
- **Roles:**
  - The Zero's OTG port is the USB host (dwc2 in host mode).
  - The Pico is a full-speed (12 Mbit/s) device on TinyUSB, which the Pico SDK includes.
  - The link uses no GPIOs.
- **Composite device**, with custom descriptors instead of `pico_stdio_usb`:

  | Interface | Use |
  |---|---|
  | CDC 0 | The framed link (below) |
  | CDC 1 | Transparent UM980 COM1 passthrough, for UPrecise and receiver firmware upgrades. This settles the "COM1 kept for firmware upgrades" note in [rp2350-feasibility.md](rp2350-feasibility.md) |
  | Reset interface (as `pico_stdio_usb` provides it) | `picotool` on the Zero reboots the Pico into BOOTSEL and loads a UF2 |

- **Framing:** reuse the COBS frames from [esp32-s3-wifi-companion.md](esp32-s3-wifi-companion.md): a type byte, a sequence number and a CRC-16, with the `HELLO`, `POS`, `DIAG`, `CFG_GET`/`CFG_SET`/`CFG_ACK` and `HEARTBEAT` types. Add these:

  | Type | Direction | Content |
  |---|---|---|
  | `NMEA` | Pico → Zero | ready-made GGA/RMC at 10 Hz, forwarded unchanged to SW Maps |
  | `RAW` (optional) | Pico → Zero | TM210, UM980 and BMP581 samples with their Pico capture timestamps, for logging and replay |

  No RTCM type is needed, because RTCM goes straight to COM2. The CRC stays: the link can be unplugged or reset in the middle of a frame.
- **Bandwidth (estimate, not yet measured):**
  - `POS`, `NMEA` and `DIAG`: about 3 kB/s
  - `RAW`: about 50 kB/s. That is TM210 raw data at 39.2 kB/s ([rp2350-feasibility.md](rp2350-feasibility.md)) plus GNSS and baro.
  - Both are well within what a full-speed CDC link carries in practice.
- **Linux side:** udev rules give the interfaces stable names by VID:PID and interface number, e.g. `/dev/gnss-link` and `/dev/um980-com1`.

## Timing
- **All capture timestamps stay on the Pico,** taken in the ISR or DMA-complete handler and from PPS. USB and Linux jitter affect only output latency, never the filter.
- **`POS` carries the GNSS time of validity,** so a consumer can compensate for latency. USB and the daemon add a few ms (estimate, not yet measured).
- **The two clocks need no sync for navigation.** The Zero's own time comes from COM2 and PPS (below), not from the Pico.

## Stratum 1 NTP server on the Zero
- **Why a direct path from the UM980:** NTP keeps working while the Pico reboots or is being debugged, and USB latency stays out of the time path.
- **COM2 ↔ Zero UART, both directions:**
  - UM980 COM2 TX → Zero RXD (GPIO15) carries NMEA time.
  - Zero TXD (GPIO14) → COM2 RX carries RTCM only (see the RTCM path).
  - COM2 stays at 460800, as [gnss-um980-config.md](gnss-um980-config.md) sets it.
  - Through COM1, the Pico enables `GPRMC COM2 1` and `GPZDA COM2 1`, giving UTC date and time once a second. That is about 0.2 kB/s out, against 1–3 kB/s of RTCM in (estimate).
- **UART choice on the Zero:**
  - `dtoverlay=miniuart-bt` moves Bluetooth to the mini UART, which leaves the PL011 (`/dev/ttyAMA0`) for the GNSS with a fixed core clock.
  - The mini UART's 8-byte FIFO is a poor fit for 460800.
  - Bluetooth traffic is small (a few kB/s), but BLE NUS and SPP running on the mini UART still needs a bench test.
- **PPS to both boards:**
  - The UM980 PPS output fans out to the Pico GPIO (as now) and to Zero GPIO18, using `dtoverlay=pps-gpio,gpiopin=18`. It shows up on the Zero as `/dev/pps0`, timestamped in the kernel.
  - The existing `CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0` fits: 1 Hz, rising edge, and no edges before time has converged.
  - Keep both branches short. A series resistor on each branch is optional.
  - Check the PPS output level and drive strength in `docs/datasheets/UM980_Datasheet.pdf`. Both boards expect 3.3 V logic.
- **Software:** gpsd reads `/dev/ttyAMA0` and `/dev/pps0`, and chrony uses it.
  - `refclock SHM 0 refid NMEA noselect offset <calibrated>`
  - `refclock PPS /dev/pps0 lock NMEA refid PPS prefer`
  - `leapsectz right/UTC`
  - `makestep` at boot, since the Zero has no RTC
  - `allow` for the Zero's Wi-Fi subnet
  - The NMEA offset is calibrated on the bench.
- **Expected accuracy (estimate, not yet measured):**
  - the Zero's clock against PPS: a few µs
  - clients over Wi-Fi: about 1 ms or worse, limited by Wi-Fi jitter
- **Stratum:** chrony serves stratum 1 only while PPS is locked. If PPS is lost, the clock free-runs on the Zero's crystal and the stratum degrades.
- **Clients:** the AgOpenGPS tablet and other devices on the Zero's Wi-Fi. The Zero also timestamps its raw logs in UTC.

## SWD from the Zero: the first build is also the debug rig
The Zero drives the Pico's SWD port, so the first hardware build needs no separate debug probe. Flashing and debugging work over Wi-Fi.
- **Wiring:** connect the Pico's SWD header (SWCLK, SWDIO, GND) to two free Zero GPIOs. Follow the Raspberry Pi wiring in `docs/RP-008276-DS-4-getting-started-with-pico.pdf` (GPIO24/25 there; check). Avoid GPIO14/15 (UART) and GPIO18 (PPS). A Zero GPIO to the Pico's RUN pin is optional.
- **OpenOCD:**
  - Use Raspberry Pi's OpenOCD build, which supports the RP2350, with the `bcm2835gpio` bit-bang adapter and `target/rp2350.cfg`.
  - Flash with `openocd ... -c "program app.elf verify reset exit"`.
  - OpenOCD serves GDB on port 3333. The PC runs `arm-none-eabi-gdb` or VS Code Cortex-Debug against `<zero>:3333` over Wi-Fi, and both M33 cores are visible.
- **Logs without pins:** SEGGER RTT through OpenOCD (`rtt setup`, `rtt server`). The Pico needs no UART for a console, and USB CDC stays free for the link. While the debugger has core 1 halted, the filter is stopped, as expected.
- **SWD or picotool:** SWD is the development path, and it also recovers a Pico with broken flash. `picotool` over the USB reset interface is the field-update path, which needs no SWD wires.

## Config storage
- **The master copy** of the versioned config struct lives on the Zero, on a small data partition outside the read-only root.
- **The Pico keeps a last-known copy in its flash,** so it starts with the right settings long before the Zero has booted. It writes that copy only when the config changes, with the filter paused. The XIP stall and `multicore_lockout` handling from [web-config-server.md](web-config-server.md) then matter only at a save.
- **At boot,** once the link is up, the Pico sends `CFG_GET`. If the reply differs from its copy, it applies the reply and re-aligns.

## Power, boot and failure
- **Supply:** one 5 V supply feeds the Zero. The Pico takes VBUS from the Zero's OTG port, or VSYS from the same 5 V rail through the diode arrangement in `docs/datasheets/RP-008304-DS-3-pico-2-w-datasheet.pdf` (power section; the plain Pico 2 is the same). Add up the Pico, UM980 and TM210 currents and check them against what the OTG port can supply.
- **Boot:**
  - The Pico is up in under a second and starts aligning.
  - The Zero needs about 20–40 s (estimate, not yet measured). Until the host opens the port (DTR), the Pico drops link output.
  - Bluetooth output, RTCM and NTP start only once the Zero is up.
- **Read-only root:** the Zero runs a read-only root (overlayfs), so power cuts can't damage the filesystem. Logs go to a separate partition or a USB stick.
- **If the Zero fails:**
  - The filter keeps running.
  - There is no wireless output and no RTCM, so RTK falls back to a standalone fix.
  - The Zero is a single point of failure for everything wireless. A Pico 2 W alone doesn't have this weakness, but it doesn't have the features either.

## Wiring
| Signal | From | To |
|---|---|---|
| USB D+, D−, VBUS, GND | Zero OTG port (host) | Pico USB (device) |
| UM980 COM2 TX → Zero RX | UM980 COM2 TX | Zero GPIO15 (RXD, PL011) |
| Zero TX → UM980 COM2 RX (RTCM only) | Zero GPIO14 (TXD) | UM980 COM2 RX |
| PPS | UM980 PPS | Pico GPIO interrupt and Zero GPIO18 |
| SWD | Pico SWCLK / SWDIO | Zero GPIO25 / GPIO24 (check) |
| Reset (optional) | Zero GPIO | Pico RUN |
| GND | common | all boards |

Check the Pico pins against the final pin map. Already used: I2C1 on GP2/GP3, GP16–21 reserved for the W5500 (on hold), the web-mode switch on GP22, plus the UART0/UART1 pins.

## Compared with the other layouts
| | Pico 2 W alone | Pico 2 + ESP32-S3 | Pico 2 + Zero 2 W |
|---|---|---|---|
| Parts | one board | two boards plus a regulator | two boards |
| Software | one firmware | two firmwares plus a link protocol | one firmware plus a Linux image and daemon |
| Time to first output | under 1 s | about 1 s | 20–40 s for wireless output (estimate) |
| Filter isolation | radio stack shares core 0 | radio on a separate chip | radio and networking on a separate board |
| Networking software | lwIP and BTstack by hand | ESP-IDF | full Linux: BlueZ, NTRIP, web server, gpsd |
| Bluetooth Classic SPP fallback | yes | no | yes |
| RTCM path | through the Pico into COM1 | through the Pico into COM1 | Zero straight into COM2 |
| Pico firmware update and debug | separate probe | separate probe (SWD from the S3 not checked) | SWD and GDB from the Zero over Wi-Fi, `picotool` |
| Raw logging | needs an SD card on the Pico | limited | to SD or a USB stick on the Zero |
| Power draw | lowest | moderate | highest |
| Power-cut robustness | no filesystem | NVS, robust | needs a read-only root |
| Wireless single point of failure | no | the S3 | the Zero |
| Stratum 1 NTP | no | no | yes |

## Bottom line
The setup works, and USB is a clean link: it needs no pins, it solves the Zero's UART shortage, and it brings OTA updates and logging. The Zero earns its place when the Linux features are wanted:
- NTRIP without the phone
- AgOpenGPS over Wi-Fi
- raw logging
- a richer web interface
- a stratum 1 time server

The costs are a Linux image to maintain, 20–40 s before wireless output, read-only root handling, more power, and the Zero as a single point of failure for wireless.

**Recommended as the first hardware build.** The Zero is the bench and debug rig: SWD flashing and GDB over Wi-Fi, RTT logs, raw logging for replay and a time reference. It is also a usable product configuration. Bench results can decide whether a later product keeps the Zero or drops to a Pico 2 W alone.

## Open points (not yet checked)
- The current the Zero's OTG port can supply to the Pico and the sensors.
- BlueZ BLE NUS plus SPP plus Wi-Fi (access point or station) running together, with 10 Hz output and no drops.
- BLE and SPP working after Bluetooth moves to the mini UART (`miniuart-bt`).
- The TinyUSB composite device (two CDCs) together with the SDK's reset interface.
- The PPS output level and its fan-out to two inputs (UM980 datasheet).
- The NMEA arrival offset for chrony.
- The bit-bang SWD clock rate on the Zero 2 W, and how long a full flash takes.
- SD card endurance with logging, or logging to a USB stick through a hub.

## Verification (when implementing)
1. **Link soak test:** 1 h of `POS` and `NMEA`, plus `RAW` at full rate. Expect zero CRC errors and no missing sequence numbers.
2. **Zero failure:** unplug or reboot the Zero mid-run. The Pico's filter timing (the bench replay from [rp2350-feasibility.md](rp2350-feasibility.md)) is unaffected, and the link recovers when the Zero re-enumerates the Pico.
3. **Corrections:**
   - RTCM from an NTRIP caster over Wi-Fi goes from the Zero's UART into COM2, and the receiver reaches RTK fixed.
   - Repeat with SW Maps over BLE.
   - Halt the Pico with GDB: RTK fixed holds, because corrections don't pass through the Pico.
4. **Firmware update:** `picotool` on the Zero updates the Pico over USB.
5. **SWD:**
   - OpenOCD on the Zero flashes the Pico and verifies it.
   - GDB from the PC over Wi-Fi halts and steps both cores.
   - RTT logs stream.
   - A Pico with erased flash is recovered.
6. **NTP:**
   - `ppstest /dev/pps0` shows 1 Hz edges only after the fix.
   - `chronyc sources` shows PPS selected (`*`) with an offset of a few µs.
   - A Wi-Fi client syncs at stratum 2.
   - Rebooting the Pico doesn't disturb chrony.
   - `UNILOGLIST` shows the two COM2 NMEA logs, and `CONFIG` shows no RXTYPE lock.
7. **Power cuts:** cut power 50 times during logging. Expect no filesystem damage.
8. **Replay:** a `RAW` log replayed through the host INSLIB `replay` matches the on-device output.
