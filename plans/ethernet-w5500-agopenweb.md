# Ethernet output with a W5500, compatible with AgOpenWeb

> **Status: on hold** (2026-11-06). This is not part of the current build. The plan is kept for when wired Ethernet / AgOpenWeb output is added, and SPI0 (GP16–21) stays reserved for it.

## Context
Add a WIZnet W5500 Ethernet module on SPI so the device sends its position over wired UDP to [AgOpenWeb](https://github.com/AgOpenGPS-Official/AgOpenWeb) (and AgOpenGPS/AgIO) on the tractor network. BLE output stays as it is. Ethernet is an extra channel, not a replacement.

## What AgOpenWeb expects
Read from AgOpenWeb's `develop` branch: `Docs/GPS_RECEIVERS.md`, `Docs/PGN.md`, `Shared/AgOpenWeb.Services/NmeaParserServiceFast.cs`, `UdpCommunicationService.cs` and `NtripClientService.cs`.

- **UDP only.** AgOpenWeb opens no serial ports. GNSS text datagrams are received on port **9999**, the module port (2211 and 2222 also work).
- **Text input, not binary.** The accepted inputs are `$PANDA`, `$PAOGI`, `$KSXT`, standard `GGA`/`GNS` + `VTG` + `HPR`/`HDT`/`THS`, and Unicore `#INSPVAXA`. Binary PGN 214 is **not** accepted as input; AgOpenWeb only sends it.
- **No hello is needed.** Any datagram starting with `$`, `#` or `%` marks its sender as the GPS source, for 10 s.
- **RTCM comes back** from AgOpenWeb's built-in NTRIP client over UDP to the GPS sender's address on port **2233**. If the sender is unknown, it goes to subnet .255:2233. The datagrams are 256 bytes, sent 25 ms apart.
- A datagram may carry several whole lines. NMEA checksum: 8-bit XOR of the bytes between `$` and `*`, two hex digits.

## Output format: `$PANDA` at 10 Hz
`$PANDA` is the AiO board's "single antenna + IMU" sentence. That matches a UM980 + TM210 INS exactly. The fields are below, numbered as in AgOpenWeb's parser:

```
$PANDA,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,age,speed,heading,roll,pitch,yaw_rate*XX
```

| # | Field | What to send | How AgOpenWeb reads it |
|---|---|---|---|
| 1 | time | UTC hhmmss.ss from the UM980 | not used for position |
| 2–3 | lat, N/S | DDMM.MMMMMMM (7 decimals ≈ 2 mm) | S negates |
| 4–5 | lon, E/W | DDDMM.MMMMMMM | W negates |
| 6 | fix | see "Fix quality" below | raw integer, GGA scale |
| 7 | sats | from the UM980 | raw |
| 8 | hdop | from the UM980 | raw |
| 9 | alt | metres | raw |
| 10 | age | age of corrections, seconds | raw |
| 11 | speed | knots | ×0.514444 → m/s |
| 12 | heading | INS yaw **×10 as an integer** (0–3599); 65535 = invalid | `int`, ×0.1 |
| 13 | roll | INS roll **×10 as an integer** | `int`, ×0.1, ignored when heading is invalid |
| 14 | pitch | degrees | raw, no scaling |
| 15 | yaw rate | deg/s | raw, no scaling |

Pitfalls:
- **Heading and roll must be integers.** A value such as `123.4` in the heading field fails to parse, and AgOpenWeb then treats the IMU as invalid.
- The sentence ID must be exactly `$PANDA`, with no talker prefix (`$GPPANDA` is not recognised).
- Send yaw rate even when it's 0, so all 15 fields are present.

**Position point:** send the INS position at the **GNSS antenna** (the solution propagated through the IMU → antenna lever arm), not at the IMU. AgOpenWeb applies its own antenna-height roll compensation and pivot offsets, so sending a point that's already been moved would correct it twice.

**Fix quality:**
- Pass the UM980 quality through (4 = RTK fixed, 5 = float, 2 = DGPS, 1 = single) while GNSS updates are fresh.
- Send 6 (dead reckoning) when GNSS has been missing for more than about 1 s while the INS is coasting.
- Send 0 when the filter isn't navigating, and while output is paused for a config change (see [web-config-server.md](web-config-server.md)). On this channel, that takes the place of the status flag.
- Send heading 65535 while the filter is aligning.

**Sign conventions (to verify on the bench):** heading clockwise from true north, roll positive with the right side down, yaw rate positive clockwise. Check each one against AgOpenWeb's display before trusting it.

**Later options:** `$PAOGI` (float degrees, dual-antenna style) or GGA+VTG+HPR, selectable in the config, for other AOG tools.

## Hardware and pins
| W5500 | Pico 2 W |
|---|---|
| MISO | GP16 (SPI0 RX) |
| CS | GP17 |
| SCK | GP18 |
| MOSI | GP19 |
| RST | GP20 |
| INT | GP21 |

- These are the same pins as WIZnet's W5500-EVB-Pico, so its examples run without changes.
- The BMP581 moves to I2C (e.g. I2C1 on GP2/GP3), so SPI0 is the W5500's alone.
- SPI clock about 30 MHz.
- The W5500 draws about 130 mA at 3.3 V when linked. Check the Pico 2 W 3V3 regulator budget, or power the module from its own regulator.
- Use a module with an integrated RJ45 and magnetics. For a tractor, use a sealed connector (e.g. M12 D-coded) on the enclosure.

## Software stack
- WIZnet `ioLibrary_Driver`, which uses the W5500's hardware TCP/IP stack and 8 sockets. No lwIP is needed for Ethernet.
- Everything runs on core 0, as in [rtos-decision.md](rtos-decision.md). The filter stays alone on core 1.
- The Wi-Fi web server ([web-config-server.md](web-config-server.md)) keeps using lwIP on the CYW43439. The two stacks are independent.
- **Network settings:**
  - Static IP 192.168.5.124 (the AOG GPS-module address), mask /24, configurable from the web config page.
  - Socket 0: UDP, sends `$PANDA` to 192.168.5.255:9999.
  - Socket 1: UDP, listens on 2233 for RTCM.
  - Link up/down is read from the W5500 PHY status register and shown on the diagnostics page.
  - The MAC address is derived from the RP2350 unique ID, with the locally administered bit set.

## RTCM over Ethernet
- Path: UDP 2233 → ring buffer → RTCM3 frame check → the `um980_tx` arbiter → UART1 TX → UM980 **COM1**. This is the same path that Bluetooth RTCM uses, and no port is locked ([gnss-um980-config.md](gnss-um980-config.md)).
- Bluetooth RTCM from the phone (SW Maps, [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)) goes into the same queue. Only one correction source may be active at a time; the arbiter takes frames from the selected source and drops the other's. The diagnostics page shows which source has delivered frames recently.
- With this in place, AgOpenWeb's NTRIP client can replace the phone as the correction source.

## Discovery
- Nothing is needed for v1, because AgOpenWeb learns the GPS source from the first `$` datagram.
- Optional later: answer the AOG scan request (PGN 202 on port 8888, reply PGN 203). Its byte layout is not verified yet.

## Cost (estimate, not yet measured)
- CPU: negligible. The traffic is 10 × ~120 B/s out plus RTCM at ~1–2 kB/s in, over 30 MHz SPI.
- Flash: about 20–30 KB for ioLibrary and the glue code. RAM: under 10 KB.

## Verification (when implementing)
1. A PC on 192.168.5.x running `nc -ul 9999` (or Wireshark) receives `$PANDA` at 10 Hz with valid checksums.
2. AgOpenWeb on the same network shows the position, RTK fixed, IMU heading and roll. Tilt and turn the unit, and check that the roll sign and heading direction match the display.
3. Turn on AgOpenWeb's NTRIP client with the phone's corrections off. RTCM bytes arrive on 2233, and the UM980 reaches RTK fixed.
4. Pull the Ethernet cable while running. Core 1 filter timing is unaffected (bench replay from [rp2350-feasibility.md](rp2350-feasibility.md)), and output resumes when the cable is plugged back in.
