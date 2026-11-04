# GNSS alternative: Septentrio mosaic-X5 instead of UM980

## Context
The current GNSS receiver is the Unicore UM980 ([rp2350-feasibility.md](rp2350-feasibility.md), [inslib-nav-suite.md](inslib-nav-suite.md)). This plan compares it with the Septentrio mosaic-X5 and lists what a switch would change in performance and firmware. It is a comparison, not a decision.

Sources: `docs/datasheets/UM980_Datasheet.pdf` (R1.0, 2022-08), `docs/datasheets/Mosaic-X5_Datasheet.pdf` (2020 brochure), and `docs/datasheets/Unicore Reference Commands Manual For N4 High Precision Products_V2_EN_R1.15.pdf`.

## Datasheet comparison
| | UM980 | mosaic-X5 |
|---|---|---|
| RTK accuracy (RMS) | 0.8 cm + 1 ppm H, 1.5 cm + 1 ppm V (1 km baseline) | 0.6 cm + 0.5 ppm H, 1 cm + 1 ppm V (<40 km baseline) |
| RTK initialisation | <5 s typ, reliability >99.9 % | 7 s |
| Standalone / DGNSS (H) | 1.5 m / 0.4 m | 1.2 m / 0.4 m |
| Velocity accuracy | 0.03 m/s | 3 cm/s |
| Max update rate | 50 Hz (after firmware upgrade) | 100 Hz |
| Latency | not specified | <10 ms (99.9 %) |
| Channels | 1408 (NebulasIV) | 448 |
| GPS | L1 C/A, L1C*, L2P(Y), L2C, L5 | L1C/A, L1C, L1PY, L2C, L2P, L5 |
| GLONASS | L1, L2 | L1, L2, L2P, L3 CDMA |
| Galileo | E1, E5a, E5b (**no E6**) | E1, E5a, E5b, E5 AltBOC, E6 ("hardware ready") |
| BeiDou | B1I, B2I, B3I, B1C, B2a, B2b* | B1I, B1C, B2a, B2I, B3I ("hardware ready") |
| Other | QZSS L1/L2/L5, SBAS | QZSS, NavIC L5, SBAS L1/L5, on-module L-band |
| Interference / robustness | JamShield, 60 dB narrowband anti-jam | AIM+ (narrow band, wide band, chirp), APME+ multipath, LOCK+ vibration/shock, IONO+ scintillation, RAIM+ integrity |
| Tracking / re-acquisition | not specified | 20 dB-Hz tracking, 1 s re-acquisition |
| Time | PPS, 20 ns time accuracy, 1 EVENT input | PPS 5 ns, 2 event inputs <20 ns |
| Interfaces | 3 UART (I2C, SPI, CAN reserved, not supported) | 4 UART up to 4 Mbps, Ethernet RMII 10/100 (external PHY), USB 2.0, SDIO logging, 2 GPIO, CAN ("hardware ready") |
| Formats | NMEA-0183, Unicore binary, RTCM 3.x in | SBF, NMEA, RINEX, RTCM 2.x/3.x, CMR |
| Other features | — | moving-base RTK, base and rover |
| Power | 480 mW typ, 3.0–3.6 V | 0.6 W typ, 1.1 W max, 3.3 V ±5 % |
| Size / weight | 22 × 17 × 2.6 mm, 1.9 g | 31 × 31 × 4 mm, 6.8 g |
| Price | low | roughly 3–4× |

\* after a firmware upgrade.

Both are single-antenna receivers. A dual-antenna variant (mosaic-H, UM982) would be needed for heading at standstill.

## Effect on the system
- **Open sky: no practical difference** for AgOpenGPS steering. Both hold 1–2 cm RTK, and the UM980 even initialises slightly faster on paper. The mosaic's ppm term is half the UM980's, which only shows on long baselines (e.g. 20 km NTRIP: 1.6 cm vs 2.8 cm + base).
- **Where the mosaic is better:** tree lines, buildings, jammers (common near roads), multipath off the cab and roof, and engine vibration. Expect fewer float or lost-fix episodes and faster recovery, and RAIM+ adds integrity against wrong fixes. This reduces how many GNSS outages the IMU has to bridge ([imu-adis16465-vs-tm210.md](imu-adis16465-vs-tm210.md) covers the IMU side).
- **Signals:** the UM980 does not track Galileo E6, so Galileo HAS is unavailable on it. The mosaic's E6 support depends on firmware (the 2020 sheet says "hardware ready"). For RTK with an NTRIP or radio base, neither matters.
- **Timing:** both receivers have a PPS output and an event input. The mosaic's two event inputs (<20 ns) can timestamp the IMU's data-ready pulse directly in GNSS time.
- **Covariance for INSLIB:** SBF `PosCovGeodetic` / `VelCovGeodetic` give a full covariance for `gnss_pos` / `gnss_vel` `Qll_ned`. That is more useful than the UM980's per-axis σ, although `gnss_pos_cov_scale` tuning is still needed.
- **Heading:** unchanged. `automotive_mode` (heading from COG) stays the main source while moving.

## Firmware impact if switched
- **Parser:** replace the Unicore binary parser (BESTNAV/PVTSLN) with SBF: `PVTGeodetic`, `PosCovGeodetic`, `VelCovGeodetic`, `ReceiverTime`, plus `EndOfPVT` to mark the epoch. SBF has CRC-16 and a fixed header, so the parsing pattern is similar.
- **Config:** Septentrio commands (`setSBFOutput`, `setDataInOut`, `setPVTMode`) instead of Unicore `CONFIG`/`LOG`.
- **Ports** ([rp2350-feasibility.md](rp2350-feasibility.md) layout):
  - UART1 stays the receiver's SBF output and command port. It could run faster, up to 4 Mbps.
  - Radio RTCM goes into a second mosaic UART with input locked to RTCM via `setDataInOut`.
  - A spare mosaic UART takes RTCM from the Pico's PIO TX (AgOpenWeb NTRIP over the W5500).
  - Optional: use the mosaic's own Ethernet and NTRIP client (needs a PHY on the carrier board), with SD logging over SDIO for field data.
- **Fix quality output:** map the SBF `Mode` field (RTK fixed/float, DGNSS, standalone) to the GGA quality field. `nav_suite` mode handling is unchanged.
- **Power:** budget 1.1 W peak at 3.3 V (≈330 mA) instead of ~150 mA.

## Bottom line
In open sky the mosaic-X5 gives the same accuracy as the UM980. It wins on robustness (interference, multipath, vibration, canopy), integrity (RAIM+), timing and interfaces, not on accuracy. It is worth it if the tractor works near tree lines, buildings or busy roads, or if jamming has been seen. It is not worth it for open-field work only. The costs are the price, an SBF parser and slightly more power.

## Verification (if hardware is bought)
1. Run both receivers on one antenna through a splitter, with the same corrections. Log fix status, σ and position side by side on open sky, along a tree line and near buildings.
2. Compare the share of time in RTK fixed, time to re-fix after canopy, and false-fix jumps against a reference trajectory.
3. Replay both logs through `external/INSLIB/tools/replay.c` with the same IMU data and compare the INS output.
