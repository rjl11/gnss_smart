# ESP32-S3 as Wi-Fi/BLE companion to a plain Pico 2

## Context
The question: can a Pico 2 with no radio use an ESP32-S3 for wireless, in place of the CYW43439 chip on the Pico 2 W? The wireless features in question are the web config page ([web-config-server.md](web-config-server.md)), BLE position output ([rp2350-feasibility.md](rp2350-feasibility.md)) and a later NTRIP client over Wi-Fi. The chip itself is covered in [esp32-s3-feasibility.md](esp32-s3-feasibility.md).

## Answer: yes, with each chip owning whole features
The S3 works best running its own firmware that owns the wireless features. Using it as a bare network adapter that the Pico drives byte by byte works less well. There are three ways to connect it:

| Approach | What runs on the S3 | What the Pico has to do | Verdict |
|---|---|---|---|
| ESP-AT (Espressif's stock firmware, controlled by AT commands) | Wi-Fi, TCP/UDP, BLE, all driven by AT commands | Parse AT replies; build the HTTP server and BLE NUS on top of AT sockets | Works, but serving web pages and BLE through AT commands is clumsy |
| ESP-Hosted (Espressif's network co-processor firmware) | Wi-Fi/BLE driver; the host sees a network interface | Host drivers exist only for ESP-IDF and Linux, **not for the Pico SDK** | Too much porting |
| **Own S3 firmware, features split between the chips** (recommended) | Web config server, BLE NUS output, NTRIP client, OTA, config storage | Send position and diagnostics, receive config, over one framed link | Simplest code on both chips; uses ESP-IDF's ready-made networking |

## Recommended split
- **Pico 2:** keeps its sensor and filter role.
  - TM210 on UART0, UM980 COM1 on UART1, BMP581 on I2C1, W5500 on SPI0.
  - The filter runs alone on core 1, so bare metal still works as planned in [rtos-decision.md](rtos-decision.md).
  - The Pico has no radio code at all.
- **ESP32-S3:** everything wireless.
  - Web config and diagnostics on `esp_http_server`.
  - BLE output with the Nordic UART Service (NUS) on NimBLE.
  - NTRIP client over Wi-Fi, with TLS if the caster needs it.
- **RTCM from NTRIP goes straight from the S3 to UM980 COM3** on one of the S3's own UARTs, so corrections never pass through the Pico.
  - RTCM over Ethernet from AgOpenWeb still arrives at the Pico's W5500, which also needs a path to COM3 ([ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)).
  - Choose one owner for COM3, or switch it in config. Only one correction source may be active at a time.
  - The radio stays wired directly to COM2, as before.

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
  | `POS` | Pico → S3 | the same fields as `$PANDA`, plus a status flag (aligning / navigating / paused) |
  | `DIAG` | Pico → S3 | the fields for the diagnostics page |
  | `CFG_GET` / `CFG_SET` / `CFG_ACK` | both | read, write and acknowledge the config struct |
  | `HEARTBEAT` | both, 1 Hz | liveness |

- **If one side stops responding:** the web page shows "Pico not responding". If the S3 is dead or missing, the Pico carries on, and sensors, filter and Ethernet output are unaffected.

## Config storage moves to the S3
- The versioned config struct lives in the S3's NVS (ESP-IDF's wear-levelled key-value store in flash).
- **At boot**, the Pico sends `CFG_GET` and applies the reply.
- **On a web save**, the S3 sends `CFG_SET`. The Pico pauses the filter, applies the settings, re-aligns and replies `CFG_ACK`.
- **The Pico never writes its own flash in normal use**, so the XIP stall and the `multicore_lockout` handling in [web-config-server.md](web-config-server.md) are no longer needed. The Pico keeps a compiled-in default config for when the S3 is missing.

## Web-mode switch and power
- The web-mode switch stays on Pico GP22, which the Pico reads at boot. The Pico drives the S3's `EN` pin from another GPIO.
- **If BLE output is needed only in web mode:** hold the S3 in reset outside web mode. It then draws about 0 mA and puts no RF near the GNSS antenna.
- **If BLE output is needed all the time:** keep the S3 running with BLE on, and start Wi-Fi only in web mode.
- **Power supply:** the S3 draws about 40–60 mA with the radio idle, and Wi-Fi transmit peaks reach about 300 mA. Give it its own 3.3 V regulator rather than the Pico's 3V3 pin.

## Wiring (Pico 2 ↔ ESP32-S3 dev board)
| Signal | Pico 2 | ESP32-S3 |
|---|---|---|
| Link, Pico TX → S3 RX | PIO UART TX (e.g. GP26) | UART1 RX (any GPIO via the GPIO matrix) |
| Link, Pico RX ← S3 TX | PIO UART RX (e.g. GP27) | UART1 TX |
| S3 enable | GP28 (output) | EN |
| RTCM to UM980 COM3 | — | UART2 TX |
| GND | GND | GND |

Check the GP26–28 choice against the final Pico pin map. Already used: I2C1 on GP2/GP3, the W5500 on GP16–21, the web-mode switch on GP22, plus the UART0/UART1 pins.

## Compared with a Pico 2 W alone
| | Pico 2 W alone | Pico 2 + ESP32-S3 |
|---|---|---|
| Parts and cost | one board, about 1 USD more than a Pico 2 | two boards, +5–15 USD, plus a regulator |
| Firmwares | one | two, plus a link protocol and version checks |
| Filter isolation | CYW43, BTstack and lwIP share core 0 with the drivers | radio stack on a separate chip |
| Networking software | lwIP and BTstack set up by hand; NTRIP with TLS and OTA take a lot of work | ESP-IDF includes an HTTP server, TLS, OTA and NVS |
| Pico flash writes | needed for config, and they stall execution from flash | not needed |
| Bluetooth Classic serial port (SPP) | possible | not possible (the S3 has BLE only) |

## Bottom line
The setup works, as long as each chip owns whole features: the S3 handles everything wireless plus the config, and the Pico handles sensors, filter and Ethernet. The cost is a second firmware and a link protocol. It's worth it if NTRIP over Wi-Fi, OTA or a richer web interface are wanted, or to keep all radio code away from the filter. If wireless is only for occasional config and BLE output, a Pico 2 W alone is simpler.

## Open points (not yet checked)
- Updating the Pico's firmware through the S3. Possible routes are SWD driven from the S3 or the RP2350 bootrom's UART boot mode, but neither has been checked.
- Whether BLE NUS and the Wi-Fi access point running together on the S3 still deliver 10 Hz output without drops. This needs a bench test.

## Verification (when implementing)
1. **Link soak test:** the Pico sends `POS` at 10 Hz and `DIAG` at 1 Hz for 1 h. Expect zero CRC errors and no missing sequence numbers.
2. **Web page and config:** the S3's web page shows live diagnostics from the Pico. A config save pauses the filter, re-aligns and resumes. After a power cycle, the config comes back from NVS.
3. **S3 failure:** hold the S3's EN low, or unplug it, while the system is running. The Pico's filter timing (bench replay from [rp2350-feasibility.md](rp2350-feasibility.md)) and its Ethernet `$PANDA` output must be unaffected.
4. **NTRIP:** with the radio disconnected, NTRIP on the S3 feeding UM980 COM3 reaches RTK fixed.
