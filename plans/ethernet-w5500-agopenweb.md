# Ethernet output with a W5500, compatible with AgOpenWeb

> **Status: on hold** (2026-11-06). This is not part of the current build. The plan is kept for when wired Ethernet / AgOpenWeb output is added, and SPI0 (GP16–21) stays reserved for it.

## Context
Add a WIZnet W5500 Ethernet module on SPI so the device sends its position over wired UDP to [AgOpenWeb](https://github.com/AgOpenGPS-Official/AgOpenWeb) (and AgOpenGPS/AgIO) on the tractor network. BLE output stays as it is. Ethernet is an extra channel, not a replacement.

## What AgOpenWeb expects
Read from AgOpenWeb's `develop` branch at commit `e68dcfee` (2026-11-10): `Docs/GPS_RECEIVERS.md`, `Docs/PGN.md`, `Plans/RTCM_FORWARDING_PLAN.md`, and in `Shared/AgOpenWeb.Services/`: `NmeaParserServiceFast.cs`, `UdpCommunicationService.cs`, `NtripClientService.cs` and `RtcmQueue.cs`.

- **UDP only.** AgOpenWeb opens no serial ports. GNSS text datagrams are received on port **9999**, the module port. Ports 2211 (GPS1) and 2222 (GPS2) are for a bridge that keeps two receivers apart; this device uses 9999.
- **Text input, not binary.** The accepted inputs are:
  - `$PANDA` (single antenna + IMU) and `$PAOGI` (dual antenna), the AiO board's sentences.
  - `$KSXT`.
  - The standard set: `GGA` or `GNS`, plus `VTG`, `HPR`, `HDT`, `THS` in any combination, grouped by epoch.
  - Unicore INS logs: `#INSPVAXA`, `#INSPVAA`, `%INSPVAXSA`, `%INSPVASA`, with the CRC-32 checked.
  - Binary PGN 214 is **not** accepted as input; AgOpenWeb only sends it.
- **Who owns the heading depends on the sentence:**
  - `$PANDA` field 12 is taken as a raw **IMU heading**. AgOpenWeb fuses it with its own fix-to-fix heading.
  - A `$PAOGI` heading and an `#INSPVAXA` azimuth are taken as ground truth, with no fusion on the host.
- **No hello is needed.** Any datagram starting with `$`, `#` or `%` marks its sender as the GPS source, for 10 s.
- **RTCM comes back** from AgOpenWeb's built-in NTRIP client on UDP port **2233**:
  - It is sent unicast to the GPS source's address. It goes to subnet .255:2233 instead when the source is unknown or the "broadcast only" setting is on, so listen for both.
  - The datagrams are at most 256 bytes, 25 ms apart on average. When the sender catches up, the gap shrinks to 10 ms. The bridge contract's ceiling is 512 bytes and a 10 ms gap.
  - The app frames the caster's stream into whole RTCM messages before queueing, but a datagram is a 256-byte chunk of that queue, so an RTCM message can span datagrams. If the mount point sends no RTCM 3, the stream is forwarded byte for byte.
- **Line format:** each datagram holds one or more whole lines, `$…*hh\r\n`. One line per datagram, sent as soon as it is complete, is preferred. NMEA checksum: 8-bit XOR of the bytes between `$` and `*`, two hex digits.

## Output format: `$PAOGI` at 10 Hz
The device sends **only `$PAOGI`**, on a UM980 and on a UM982 alike. `$PANDA` is not sent (decision 2026-11-10): AgOpenWeb would treat its heading as a raw IMU heading and fuse it with its own fix-to-fix heading, although ours is already the INS yaw. A `$PAOGI` heading is taken as it is. The AgOpenWeb-HAT daemon makes the same choice: it sends `$PAOGI` whenever the receiver has a dual heading or an INS (https://github.com/chriskinal/AgOpenWeb-HAT, `core/src/nav.cpp`, commit `e4bd9ef`, read 2026-11-10). The fields are below, numbered as in AgOpenWeb's parser:

```
$PAOGI,time,lat,N/S,lon,E/W,fix,sats,hdop,alt,age,speed,heading,roll,pitch,yaw_rate*XX
```

| # | Field | What to send | How AgOpenWeb reads it |
|---|---|---|---|
| 1 | time | UTC hhmmss.ss from the UM980 | not used for position |
| 2–3 | lat, N/S | DDMM.MMMMMMM (7 decimals ≈ 2 mm) | S negates |
| 4–5 | lon, E/W | DDDMM.MMMMMMM | W negates |
| 6 | fix | see "Fix quality" below | raw integer, GGA scale |
| 7 | sats | from the UM980 | raw |
| 8 | hdop | from the UM980 | raw |
| 9 | alt | metres **above mean sea level**, as in GGA: the INS ellipsoid height minus the receiver's undulation | raw |
| 10 | age | age of corrections, seconds. **Leave it empty when there are no corrections** | raw; empty shows "—" instead of 0.0 |
| 11 | speed | knots | ×0.514444 → m/s |
| 12 | heading | INS yaw in degrees, float (0–359.99). **Empty while the heading is not valid** | float, taken as the dual-antenna heading with no host fusion. Empty = no heading this epoch, and AgOpenWeb falls back to its fix-to-fix heading |
| 13 | roll | INS roll in degrees, float. Empty while the attitude is not valid | float. Empty = no roll. Read whether or not the heading is present |
| 14 | pitch | degrees | raw, no scaling |
| 15 | yaw rate | deg/s | raw, no scaling |

Pitfalls:
- **Never send 0 for an unknown heading.** 0 is read as north, and AutoSteer would steer on it. Leave the field empty. `$PAOGI` has no 65535 sentinel; that belongs to `$PANDA`.
- **Heading and roll are plain degrees**, not the ×10 integers of `$PANDA`.
- **Number formats that are known to work** (the AgOpenWeb-HAT daemon's `$PAOGI`, `core/src/nav.cpp`): time as `hhmmss.s`, minutes of latitude and longitude with 6 decimals, altitude with 3 decimals, heading with 1 decimal, roll and yaw rate with 2, pitch as a whole number. Round the coordinates in whole units of the last digit before splitting degrees and minutes, so 59.9999999' carries into the degrees and never prints as 60. We use 7 decimals of minutes for 2 mm resolution, which the parser also accepts.
- The sentence ID must be exactly `$PAOGI`, with no talker prefix.
- All 15 fields must be present, so keep the commas of empty fields and send yaw rate even when it's 0.
- **AgOpenWeb settings to check on the bench:** `$PAOGI` counts as a dual-antenna source, so its heading source must be set to dual, and its "Dual Heading Offset" stays 0 because the INS yaw is already the vehicle heading. The AiO v4.5 firmware at https://github.com/renatorgr/Boards (README, "Output to the program", read 2026-11-10) sends `$PAOGI` with AgOpenGPS's heading source set to "Dual", which supports this; AgOpenWeb's own setting is still to be checked.

**Position point:** send the INS position at the **GNSS antenna** (the solution propagated through the IMU → antenna lever arm), not at the IMU. AgOpenWeb applies its own antenna-height roll compensation and pivot offsets, so sending a point that's already been moved would correct it twice.

**Fix quality:**
- Pass the UM980 quality through (4 = RTK fixed, 5 = float, 2 = DGPS, 1 = single) while GNSS updates are fresh.
- Send 6 (dead reckoning) when GNSS has been missing for more than about 1 s while the INS is coasting. AgOpenWeb's fix validator has a minimum quality that decides whether a quality-6 fix is used (`Plans/GPS_RECEIVER_SENTENCES_PLAN.md`); check its setting on the bench.
- Send 0 when the filter isn't navigating, and while output is paused for a config change (see [web-config-server.md](web-config-server.md)). On this channel, that takes the place of the status flag.
- Send an empty heading field while the filter is aligning or its yaw is not valid, and an empty roll field while the attitude is not valid.

**Sign conventions (to verify on the bench):** heading clockwise from true north, roll positive with the right side down, yaw rate positive clockwise. Check each one against AgOpenWeb's display before trusting it. The AgOpenGPS V4.1 Teensy firmware (https://github.com/gunicsba/AgOpenGPS_Boards, `TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1/zHandlers.ino`, commit `f57c395`, read 2026-11-10) documents the same conventions for these sentences: roll positive with the right side down, pitch positive nose up, altitude always in metres above mean sea level. It writes the `$PAOGI` heading and roll with two decimals and sends a yaw rate of 0.

**Not planned:** a choice of output sentence. `#INSPVAXA` (the Unicore INS log AgOpenWeb decodes for the UM981) would also hand over the INS solution as it is, but it has no HDOP or correction age and needs the Unicore header and a CRC-32. GGA+VTG+HPR would serve other AOG tools. Neither is needed while `$PAOGI` works. For reference, the AiO v4.5 firmware (same repo) switches to the standard set (`GGA`, `VTG`, `THS`, and `HPR` from an IMU talker, one line per datagram) while AgOpenWeb sends its hello, and to `$PAOGI` / `$PANDA` otherwise.

**Later option: the lever arm from AgOpenWeb.** AgOpenWeb sends the vehicle geometry (antenna and IMU positions) in PGN 247. The AgOpenWeb-HAT daemon turns it into a UM981's INS lever arm (`core/include/aow/ins_setup.h`), only with steering off and the vehicle stopped, and only when it differs by more than 1 cm. The same message could fill our `gnss_leverarm_b` instead of typing it on the web config page. Its frame is X right, Y forward, Z up, so it needs converting to FRD. Not planned for the first Ethernet build.

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
  - Socket 0: UDP, sends `$PAOGI` to 192.168.5.255:9999.
  - Socket 1: UDP, listens on 2233 for RTCM, unicast and broadcast. Datagrams are up to 512 bytes, so give the socket at least a 2 KB RX buffer.
  - Link up/down is read from the W5500 PHY status register and shown on the diagnostics page.
  - The MAC address is derived from the RP2350 unique ID, with the locally administered bit set.

## RTCM over Ethernet
- AgOpenWeb's datagrams are chunks of the RTCM stream, not whole messages, so frames are reassembled across datagrams, as on the Bluetooth path.
- Path: UDP 2233 → ring buffer → RTCM3 frame check → the `um980_tx` arbiter → UART1 TX → UM980 **COM1**. This is the same path that Bluetooth RTCM uses, and no port is locked ([gnss-um980-config.md](gnss-um980-config.md)).
- Bluetooth RTCM from the phone (SW Maps, [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)) goes into the same queue. Only one correction source may be active at a time; the arbiter takes frames from the selected source and drops the other's. The diagnostics page shows which source has delivered frames recently.
- With this in place, AgOpenWeb's NTRIP client can replace the phone as the correction source.

## Discovery
- Nothing is needed for v1, because AgOpenWeb learns the GPS source from the first `$` datagram.
- Optional later: answer the AOG scan request on port 8888 (layouts from `Docs/PGN.md`; every PGN starts `80 81`):
  - Scan request, PGN 202 (0xCA): source 0x7F, length 3, data `202, 202, 5`, CRC.
  - Reply, PGN 203 (0xCB): source 0x78 (120, the GPS module), length 7, data = the four IP bytes and the first three subnet bytes, CRC.
  - Not yet tried against AgOpenWeb.

## Cost (estimate, not yet measured)
- CPU: negligible. The traffic is 10 × ~120 B/s out plus RTCM at ~1–2 kB/s in, over 30 MHz SPI.
- Flash: about 20–30 KB for ioLibrary and the glue code. RAM: under 10 KB.

## Verification (when implementing)
1. A PC on 192.168.5.x running `nc -ul 9999` (or Wireshark) receives `$PAOGI` at 10 Hz with valid checksums. AgOpenWeb's `Tools/nmea-capture.py` records the stream with arrival times, and `Tools/nmea-replay.py` plays a capture back, so the sentence can be tested without the hardware.
2. AgOpenWeb on the same network shows the position, RTK fixed, the heading (as a dual heading) and roll. While the filter is aligning, the heading field is empty and AgOpenWeb reports the dual heading as missing instead of showing north. Its System Data card (Network IO → GPS → More) shows the source as `192.168.5.124 · 9999`, the GPS rate at 10 Hz, and zero under "Not accepted" and "Lines joined". Tilt and turn the unit, and check that the roll sign and heading direction match the display.
3. Turn on AgOpenWeb's NTRIP client with the phone's corrections off. RTCM bytes arrive on 2233, and the UM980 reaches RTK fixed.
4. Pull the Ethernet cable while running. Core 1 filter timing is unaffected (bench replay from [rp2350-feasibility.md](rp2350-feasibility.md)), and output resumes when the cable is plugged back in.
