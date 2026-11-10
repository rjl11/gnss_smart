# ESP32-S3 as Wi-Fi/BLE companion to a plain Pico 2

## Context
The question: can a Pico 2 with no radio use an ESP32-S3 for wireless, in place of the CYW43439 chip on the Pico 2 W? The wireless features in question are the web config page ([web-config-server.md](web-config-server.md)), and the Bluetooth link to SW Maps on Android, which carries NMEA out and RTCM corrections in ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)). The chip itself is covered in [esp32-s3-feasibility.md](esp32-s3-feasibility.md). The Linux alternative, a Raspberry Pi Zero 2 W over USB, is in [pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md).

## Answer: yes, with each chip owning whole features
The S3 works best running its own firmware that owns the wireless features. Using it as a bare network adapter that the Pico drives byte by byte works less well. There are three ways to connect it:

| Approach | What runs on the S3 | What the Pico has to do | Verdict |
|---|---|---|---|
| ESP-AT (Espressif's stock firmware, controlled by AT commands) | Wi-Fi, TCP/UDP, BLE, all driven by AT commands | Parse AT replies; build the HTTP server and BLE NUS on top of AT sockets | Works, but serving web pages and BLE through AT commands is clumsy |
| ESP-Hosted (Espressif's network co-processor firmware) | Wi-Fi/BLE driver; the host sees a network interface | Host drivers exist only for ESP-IDF and Linux, **not for the Pico SDK** | Too much porting |
| **Own S3 firmware, features split between the chips** (recommended) | Web config server, BLE NUS link to SW Maps (NMEA out, RTCM in), OTA, config storage | Send position and diagnostics, receive config, over one framed link | Simplest code on both chips; uses ESP-IDF's ready-made networking |

## Recommended split
- **Pico 2:** keeps its sensor and filter role.
  - TM210 on UART0, UM980 COM1 on UART1, BMP581 on I2C1. SPI0 stays reserved for the W5500, which is on hold ([ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)).
  - The filter runs alone on core 1, so bare metal still works as planned in [rtos-decision.md](rtos-decision.md).
  - The Pico has no radio code at all.
- **ESP32-S3:** everything wireless.
  - Web config and diagnostics on `esp_http_server`.
  - BLE link to SW Maps with the Nordic UART Service (NUS) on NimBLE: NMEA out, RTCM in.
  - Optional later: an NTRIP client over Wi-Fi.
- **RTCM from SW Maps goes from the S3 over the link to the Pico**, and the Pico writes it to UM980 COM1 through its `um980_tx` arbiter, as in [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md) and [gnss-um980-config.md](gnss-um980-config.md). Only COM1 is used, and no UM980 port is locked (`CONFIG RXTYPE` is never sent, because of the risk of bricking). There is no correction radio.
  - **No Bluetooth Classic SPP fallback on the S3.** If SW Maps' BLE driver doesn't send RTCM, this setup can't use the SPP route that the Pico 2 W has.
  - When the W5500 comes back from on hold, RTCM from AgOpenWeb arrives at the Pico and goes into the same COM1 arbiter. Only one correction source may be active at a time.
  - S3 → Pico link traffic grows by the RTCM rate (about 1–3 kB/s), which the 921600 link carries easily.

## Link between the Pico and the S3
- **Data volume is small:**
  - Pico to S3: about 10 Hz × ~150 B of position plus 1 Hz × ~1 KB of diagnostics, roughly 2.5 kB/s.
  - S3 to Pico: only config changes and commands.
- **Recommended link: a PIO UART on the Pico at 921600 baud, 8N1.** Both of the Pico's hardware UARTs are taken. Two PIO state machines (TX and RX) with DMA give a full-duplex UART. At this rate the link is about 10 % loaded.
- **If more bandwidth is ever needed:** use SPI1 with the Pico as master and the S3 as SPI slave, plus a "data ready" GPIO from the S3 (the same pattern ESP-Hosted uses).
- **Framing:** COBS-encoded frames, each with a type byte, a sequence number and a CRC-16. Message types:

  | Type | Direction | Content |
  |---|---|---|
  | `HELLO` | both, at boot | firmware and protocol version, so mismatched firmwares are detected |
  | `POS` | Pico → S3 | the same fields as `$PANDA` (defined in the on-hold Ethernet plan), plus the `nav_suite` mode (NONE / ATTITUDE_ONLY / COASTING / FULL) and a "paused" flag |
  | `DIAG` | Pico → S3 | the fields for the diagnostics page |
  | `CFG_GET` / `CFG_SET` / `CFG_ACK` | both | read, write and acknowledge the config struct |
  | `HEARTBEAT` | both, 1 Hz | liveness |

- **If one side stops responding:** the web page shows "Pico not responding". If the S3 is dead or missing, the Pico carries on, and sensors and filter are unaffected (as is Ethernet output, once it is back from on hold).

## Config storage moves to the S3
- The versioned config struct lives in the S3's NVS (ESP-IDF's wear-levelled key-value store in flash).
- **At boot**, the Pico sends `CFG_GET` and applies the reply.
- **On a web save**, the S3 sends `CFG_SET`. The Pico pauses the filter, applies the settings, re-aligns and replies `CFG_ACK`.
- **The Pico never writes its own flash in normal use**, so the XIP stall and the `multicore_lockout` handling in [web-config-server.md](web-config-server.md) are no longer needed. The Pico keeps a compiled-in default config for when the S3 is missing.

## Web-mode switch and power
- The web-mode switch stays on Pico GP22, which the Pico reads at boot. The Pico drives the S3's `EN` pin from another GPIO.
- **The S3 runs all the time with BLE on**, because RTK corrections arrive over BLE. Wi-Fi starts only in web mode. Holding the S3 in reset is only useful for a test without corrections.
- **Power supply:** the S3 draws about 40–60 mA with the radio idle, and Wi-Fi transmit peaks reach about 300 mA. Give it its own 3.3 V regulator rather than the Pico's 3V3 pin.

## Wiring (Pico 2 ↔ ESP32-S3 dev board)
| Signal | Pico 2 | ESP32-S3 |
|---|---|---|
| Link, Pico TX → S3 RX | PIO UART TX (e.g. GP26) | UART1 RX (any GPIO via the GPIO matrix) |
| Link, Pico RX ← S3 TX | PIO UART RX (e.g. GP27) | UART1 TX |
| S3 enable | GP28 (output) | EN |
| GND | GND | GND |

Check the GP26–28 choice against the final Pico pin map. Already used: I2C1 on GP2/GP3, GP16–21 reserved for the W5500 (on hold), the web-mode switch on GP22, plus the UART0/UART1 pins.

## Compared with a Pico 2 W alone
| | Pico 2 W alone | Pico 2 + ESP32-S3 |
|---|---|---|
| Parts and cost | one board, about 1 USD more than a Pico 2 | two boards, +5–15 USD, plus a regulator |
| Firmwares | one | two, plus a link protocol and version checks |
| Filter isolation | CYW43, BTstack and lwIP share core 0 with the drivers | radio stack on a separate chip |
| Networking software | lwIP and BTstack set up by hand; NTRIP with TLS and OTA take a lot of work | ESP-IDF includes an HTTP server, TLS, OTA and NVS |
| Pico flash writes | needed for config, and they stall execution from flash | not needed |
| Bluetooth Classic serial port (SPP) | possible: the fallback if SW Maps' BLE path doesn't send RTCM | not possible (the S3 has BLE only), so there's no fallback |

## Bottom line
The setup works, as long as each chip owns whole features: the S3 handles everything wireless plus the config, and the Pico handles sensors and filter (plus Ethernet, once it is back from on hold). The cost is a second firmware and a link protocol. It's worth it if OTA or a richer web interface are wanted, or to keep all radio code away from the filter. Now that corrections come from SW Maps over Bluetooth, a Pico 2 W alone is simpler, and it keeps the Classic SPP fallback that the S3 lacks.

## Open points (not yet checked)
- Updating the Pico's firmware through the S3. Possible routes are SWD driven from the S3 or the RP2350 bootrom's UART boot mode, but neither has been checked.
- Whether BLE NUS and the Wi-Fi access point running together on the S3 still deliver 10 Hz output without drops. This needs a bench test.

## Verification (when implementing)
1. **Link soak test:** the Pico sends `POS` at 10 Hz and `DIAG` at 1 Hz for 1 h. Expect zero CRC errors and no missing sequence numbers.
2. **Web page and config:** the S3's web page shows live diagnostics from the Pico. A config save pauses the filter, re-aligns and resumes. After a power cycle, the config comes back from NVS.
3. **S3 failure:** hold the S3's EN low, or unplug it, while the system is running. The Pico's filter timing (bench replay from [rp2350-feasibility.md](rp2350-feasibility.md)) must be unaffected, and so must its Ethernet `$PANDA` output once Ethernet is back from on hold.
4. **Corrections:** SW Maps connects to the S3 over BLE with its NTRIP client on. RTCM goes from the S3 over the link to the Pico and into UM980 COM1, and the receiver reaches RTK fixed.
