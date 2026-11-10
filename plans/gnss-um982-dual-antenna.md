# GNSS variant: UM982 dual-antenna receiver (GNSS compass)

## Context
The UM980 has one antenna, so the only GNSS heading source is course over ground (`automotive_mode`, [inslib-nav-suite.md](inslib-nav-suite.md)). Course over ground has three problems:
- It is undefined at standstill and below the minimum speed (2 m/s by default).
- It is wrong by 180° when reversing. INSLIB down-weights short reversing, but sustained reversing still drags yaw (comment on `ins_fuse_gnss_course_yaw()` in `external/INSLIB/src/ins.c`).
- After power-on, yaw stays unknown until the tractor drives forward.

The Unicore UM982 runs the same N4 firmware family and command set as the UM980 and has the same three UARTs. It also has a second antenna input (ANT2) and outputs the heading of the ANT1 → ANT2 baseline. That is a real heading, valid at standstill and when reversing.

Decisions fixed by the user:
- The UM982 is a **variant alongside the UM980, not a replacement**. The firmware finds out at boot which receiver is fitted and uses GNSS heading only on a UM982. A UM980 build behaves exactly as described in [gnss-um980-config.md](gnss-um980-config.md).
- Heading mode is **`CONFIG HEADING VARIABLELENGTH`**: the receiver does not hold the baseline length fixed.

Sources:
- `docs/Unicore Reference Commands Manual For N4 High Precision Products_V2_EN_R1.15.pdf`. Section numbers below refer to it.
- `docs/datasheets/um982-user-manual-en-r1-1.pdf` (UM982 hardware manual R1.1, 2022-09). Section and table numbers marked "HW" refer to it.
- `docs/datasheets/UM980_Datasheet.pdf`, for the comparison with the UM980.

Related:
- [gnss-um980-config.md](gnss-um980-config.md): the state machine this plan extends.
- [inslib-nav-suite.md](inslib-nav-suite.md): the filter that takes the heading.
- [rp2350-feasibility.md](rp2350-feasibility.md): port layout, unchanged.
- [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md): NMEA out, RTCM in.
- [gnss-mosaic-x5-vs-um980.md](gnss-mosaic-x5-vs-um980.md): mentions the UM982 as the dual-antenna option.

## Facts from the manual
- **Identification:** `VERSIONB` product type **17 = UM982** (18 = UM980) (§7.5.2). `MODE ROVER` needs UM982 build ≥ 7650 (UM980: ≥ 7923) (§3.4).
- **Heading is on by default** on dual-antenna receivers (§1.3). It is the angle clockwise from true north to the baseline from the master antenna (ANT1) to the slave antenna (ANT2).
- **`CONFIG HEADING`** (§4.18):
  - Modes: `FIXLENGTH` (default), `VARIABLELENGTH`, `STATIC`, `LOWDYNAMIC`, `TRACTOR`.
  - `LENGTH <cm> <tol cm>` sets a fixed baseline. It isn't used here.
  - `RELIABILITY 1..4` sets the heading reliability threshold. The default is 3.
- **`CONFIG HEADING OFFSET <heading> <pitch>`** (§4.19) adds an offset to the HEADING, GPTHS and HEADING2 outputs. The default is 0.
- **`CONFIG SMOOTH HEADING <epochs>`** (§4.49) works on dual-antenna products only. It is not used, because the filter does its own smoothing and smoothing adds lag.
- **`RESET`** (§8.2) has `RTK2` (slave antenna RTK) and `HEADING` options to restart the heading solution.
- **`UNIHEADINGB`** (message ID 972, §7.5.99) contains:
  - Solution status and position type.
  - Baseline length [m], heading 0–360° and pitch ±90°.
  - Heading σ and pitch σ [deg], the base station ID and satellite counts.
  - The note under the table: if the receiver's own INS is enabled, `pos type` can be `INS`, and then the heading comes from the INS, not from the antennas.
- **`HEADINGSTATUSB`** (ID 521, §7.5.43) echoes the `CONFIG HEADING LENGTH` setting. It isn't needed in "Run".
- **Signal groups** (§4.48, Table 4-61):
  - The UM982 takes two values, one for the master antenna and one for the slave. The default is **SG 4 5**.
  - **SG 3 6** is also supported. With it, 20 Hz positioning needs build ≥ 10979, and then only "under specific log requests".
  - The UM980 plan's `CONFIG SIGNALGROUP 2` is **not** a UM982 value.
- **Rate and accuracy (HW Table 1-1):**
  - 20 Hz positioning and heading, 20 Hz raw observations.
  - Heading accuracy is **0.1° RMS for a 1 m baseline**. It scales roughly with 1/baseline, so a 2 m bar gives about 0.05° (estimate, not yet measured).
  - RTK accuracy is the same as the UM980: 0.8 cm + 1 ppm H, 1.5 cm + 1 ppm V.
  - Unlike the UM980, the UM982 has no 50 Hz option. This project only uses 20 Hz.
- **Everything else in the UM980 plan holds for the UM982:** `FRESET`, `MODE ROVER SURVEY`, `CONFIG PSRVELDRPOS`, `CONFIG PPS`, `CONFIG COMx`, `BESTNAVB`, `STADOPB`, `UNILOGLIST`, `SAVECONFIG` and `RXTYPE AUTO`. `BESTNAV` refers to the master antenna (ANT1); the slave antenna's position is in a separate log, `BESTNAVH` (§7.5.20).

## Hardware
- **Antennas:** ANT1 (master) and ANT2 (slave), both roof-mounted with a clear sky view.
  - A longer baseline gives a more accurate heading. The receiver doesn't fix the length (VARIABLELENGTH), but the mount should still be rigid. See "VARIABLELENGTH and the mount" below.
  - Record the baseline direction in the body frame (`baseline_yaw_b`, ANT1 → ANT2, measured from the body x-axis) and the ANT1 lever arm.
  - Mounted across the cab (left → right), `baseline_yaw_b` is +90°.
- **Lever arm:** `gnss_leverarm_b` in `ins_measurements_t` is ANT1's, because BESTNAV is the master antenna position.
- **Ports: no change.**
  - UART1 ↔ COM1 at 460800 carries commands, binary logs out and RTCM in.
  - COM3 is the module's USB-C port, left for a PC. The COM2 pins are not connected, except on the Zero 2 W companion, which uses them as described in [gnss-um980-config.md](gnss-um980-config.md). All three are set to 460800.
  - No port is ever locked (`CONFIG RXTYPE` is never sent). PPS goes to the same GPIO.
- **Ready-made modules, same connections.** The UM980 and the UM982 both come on ready-made modules with the same connections: USB-C (COM3, through a CH340 bridge), COM1 RX/TX and COM2 RX/TX on pins, PPS, 5 V and GND. The UM980 module's mapping is assumed to match the UM982's; check it once. The Pico wiring is therefore identical for both, and swapping receivers is a plug change, not a board change. The UM982 module adds a second antenna connector (ANT2).
  - The bare UM982 (48-pin LGA) is not pin-compatible with the bare UM980 (54-pin LGA) (HW Table 1-1, §2.1), but the modules hide that. The pin-level rules in HW §3 (V_BCKP, BIF pull-ups, antenna feed, power-on ramp) are the module maker's job.
  - The modules have no RESET_N pin, so the firmware can't hard-reset a stuck receiver. It uses `RESET` over COM1, and power-cycling the 5 V would need a load switch (open question).
- **Power:** the module takes 5 V. The bare UM982 draws 180 mA typical and **300 mA maximum** at 3.3 V, 600 mW (HW Table 2-4), against 480 mW typical for the UM980. The 600 mW figure is for 10 Hz PVT + 10 Hz RTK + 10 Hz heading, so at 20 Hz expect more. The module's regulator and the active antennas add to that, so measure the 5 V current of each module (estimate, not yet measured).
- **Logic level:** check on the bench that the module's COM1 TX and PPS swing 3.3 V, not 5 V, before wiring them to Pico GPIO.
- **Slave antenna signals:** ANT2 tracks fewer signals than ANT1: BDS B1I/B2I/B3I, GPS L1 C/A/L2C, GLONASS L1/L2, Galileo E1/E5b, QZSS L1/L2 (HW Table 1-1). The slave antenna has no L5/E5a. That matters for heading only; position comes from ANT1.

## Detection and configuration (core 0, `gnss_um980_cfg` extended)
The state machine in [gnss-um980-config.md](gnss-um980-config.md) stays as it is, except for these changes. Renaming the module to `gnss_unicore_cfg` is left to the implementation.

1. **Identify** accepts type 18 (UM980, build ≥ 7923) or 17 (UM982, build ≥ 7650). The model goes into the diagnostics and decides which target table applies. Any other type is reported as "unsupported receiver".
2. **Target table, per model:**

| Item | UM980 | UM982 |
|---|---|---|
| Signals | `CONFIG SIGNALGROUP 2` | `CONFIG SIGNALGROUP 3 6` on build ≥ 10979 (as the module ships, see the bench results), else `4 5`. Confirm 20 Hz on the bench |
| Heading mode | — | `CONFIG HEADING VARIABLELENGTH` |
| Heading reliability | — | default (3), not sent |
| Heading offset | — | not sent (stays 0). The firmware applies `baseline_yaw_b` itself |
| Logs | `BESTNAVB COM1 0.05`, `STADOPB COM1 1` | the same, plus `UNIHEADINGB COM1 0.05` |
| Everything else | as in the UM980 plan | same |

3. **Fast-path check:** on a UM982, the `CONFIG` reply must also contain the HEADING VARIABLELENGTH line, and `UNILOGLIST` must show three logs (five with the Zero 2 W companion).
4. **Provision:** the same steps, with the UM982 rows. `CONFIG HEADING VARIABLELENGTH` goes with the step 4 settings (after the signal-group reboot, before `SAVECONFIG`). `UNIHEADINGB` goes with the logs in step 6.
5. **Bandwidth:** UNIHEADINGB is about 24 + 44 + 4 B. At 20 Hz that is about 1.4 kB/s, so COM1 TX carries about 4.5 kB/s in total, about 10 % of 460800.
6. **Where the offset is stored:** `baseline_yaw_b` is an install setting, stored in Pico flash and editable on the web config page ([web-config-server.md](web-config-server.md)). It is not stored in the receiver, so swapping a receiver doesn't lose it.

## VARIABLELENGTH and the mount
In VARIABLELENGTH mode the receiver doesn't check the baseline length against a fixed value. That trades one integrity check for robustness when the length isn't known. But the INS treats the GNSS heading as the heading of the body, so the baseline must stay rigidly fixed to the IMU even though its length isn't configured.
- **Gate in the parser:** keep a running median of the reported baseline length. Mark the heading invalid when an epoch's length differs from it by more than a set tolerance (start at 5 cm, tune on the bench). A sudden change means a wrong fix or a moved antenna.
- If ANT2 is ever placed on something that moves relative to the cab (an implement, for example), its heading is not the tractor heading and must not be fed to the INS.
- The manual also lists `TRACTOR`, but its description is unclear in the extracted table. Compare it once on the bench against VARIABLELENGTH. That is an open question; VARIABLELENGTH stays the decision.

## INSLIB integration
`ins_measurements_t` already has an absolute yaw input made for this (`external/INSLIB/src/ins.h`, `ins_meas_yaw_t` and its comment "dual-antenna GNSS heading"):
- **`m.yaw.yaw_rad`** = wrap(heading_deg − `baseline_yaw_b`) × π/180. The heading is clockwise from true north, which is the same as NED yaw. Pitch isn't used at first.
- **`m.yaw.stddev_rad`:** UNIHEADING's heading σ in radians, with a floor (start at 0.1° × 1 m / baseline length from the datasheet figure, tune from logs). Receiver σ is usually optimistic.
- **`m.yaw.is_valid`** is true only when all of these hold:
  - The solution status is SOL_COMPUTED.
  - The position type is a fixed-ambiguity type (e.g. `NARROW_INT`; confirm the values on the bench).
  - The position type is not `INS`.
  - The baseline-length gate passes.
  - The log is no older than INSLIB's 500 ms limit (`INS_MAX_DELAY_MS`).
- **`m.yaw_delay_ms` / `yaw_delay_us`:** the same as the GNSS epoch, from the log's GNSS time against the PPS-anchored capture time.
- **One epoch:** UNIHEADINGB and BESTNAVB with the same GNSS time go to core 1 in the same epoch, through the same core 0 → core 1 queue. Core 1 only gets a filled `m.yaw`.
- **`automotive_mode` off on a UM982.** INSLIB fuses the course over ground and the external yaw independently. On a UM982, course yaw adds no information and still pulls the wrong way when reversing. It is an `ins_options_t` setting, so it is fixed at `nav_suite_init()`. Core 1 therefore waits for the model from step 1 (Identify) before init. Identify comes within about a second of the receiver answering, well before the first fix.
  - **The wait is bounded.** If no receiver has been identified 10 s after boot (a full baud scan takes about 4 s), core 1 calls `nav_suite_init()` anyway with the UM980 setting (`automotive_mode` true), so a missing or dead receiver still leaves ATTITUDE_ONLY mode. If a UM982 is identified later, core 1 re-initialises the filter with `automotive_mode` off (re-align, then navigate), as a config change does ([web-config-server.md](web-config-server.md)).
  - What is lost: no heading aiding while the dual-antenna heading is invalid but the position is good (e.g. ANT2 shadowed). The gyro holds yaw over such gaps.
  - Field logs should show how often that happens. If it does, revisit: feed course yaw from the firmware side as a second `m.yaw` source only when the UNIHEADING is invalid and the tractor is moving forward.
- **Auto-init:** with `auto_init` true, the INS bootstraps yaw from the external yaw measurement (`ins.h`, auto-init comment). On a UM982, the yaw is then known before the tractor moves.
- **Mag:** unchanged (not fed to the INS).

## Outputs
- **NMEA to SW Maps:** add `$GNHDT` (true heading, from the INS yaw) at 10 Hz, only once the INS heading is valid. SW Maps and other apps show it as the heading. GGA/RMC are unchanged.
- **`$PANDA` / `$PAOGI`** (when Ethernet returns, [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)): the heading field already comes from the INS yaw. With a UM982, `$PAOGI` (the dual-antenna sentence) becomes the natural option.
- **Diagnostics page:** shows the receiver model, the heading solution status and position type, the baseline length and its median, heading σ, the count of gated epochs, and `baseline_yaw_b`.

## Core split
The same as for the UM980 ([rtos-decision.md](rtos-decision.md)):
- On core 0: UNIHEADINGB parsing, the baseline gate and the offset.
- On core 1: nothing new except reading `m.yaw` and the per-model `automotive_mode` choice at init.

## Bench results (2026-11-06, UM982 module on a PC over USB-C)
The module was connected to a PC through its USB-C port (CH340 USB-serial bridge, Windows COM10). Only read-only queries and one-shot logs were sent, and nothing was saved. The antennas weren't connected or had no sky view: BESTNAV and UNIHEADING reported `INSUFFICIENT_OBS`, so heading, rate and fix tests are still open.

- **Identity:** `"UM982","R4.10Build18975"`, product type 17. Build 18975 is above every build limit in this plan (7650 for `MODE ROVER`, 10979 for 20 Hz on SG 3 6).
- **The USB-C port is COM3.** A one-shot `VERSIONA COM3` arrived on the USB port, and `VERSIONA COM1` / `VERSIONA COM2` didn't. The module's only active log is `KSXT COM3 0.1`, and that is what streams on the USB port. The user confirmed that the module's two UART pin pairs are **COM1 and COM2**. The port assignment in all plans follows from this: Pico on COM1, Zero 2 W companion on COM2, USB-C (COM3) left for a PC.
- **Command replies confirmed:**
  - OK: `$command,<cmd>,response: OK*cs`.
  - Error: `$command,<cmd>,response: PARSING FAILED GRAMMAR ERROR,<token>*cs`, for example for `VERSIONA COM1 ONCE`. A one-shot log is requested as `<LOG>A <port>` with no trigger word.
- **`CONFIG CMDFORMAT 0`** is listed on the UM982, although the manual doesn't list the command for the UM980. Leave it at 0 (no checksum).
- **As the module ships** (`CONFIG` and `MODE` replies):

| Item | As shipped | Target in this plan |
|---|---|---|
| Signals | `SIGNALGROUP 3 6` | see the target table: `3 6` on build ≥ 10979 |
| Heading | `HEADING VARIABLELENGTH`, `HEADING RELIABILITY 3` | already matches |
| Mode | `MODE ROVER UAV DEFAULT` | `MODE ROVER SURVEY` |
| PPS | `PPS DISABLE` | `PPS ENABLE GPS POSITIVE 100000 1000 0 0` |
| Ports | COM1, COM2, COM3 at 460800 | already matches |
| Logs | `KSXT COM3 0.1` | `UNLOG`, then BESTNAVB, STADOPB, UNIHEADINGB on the Pico's port |
| Other | `ANTENNA POWERON`, `SBAS DISABLE`, `PPP ENABLE E6-HAS`, `RTK TIMEOUT 60`, `ANTIJAM AUTO` | not touched by this plan |

- **Consequences:**
  - A new module fails the fast-path check (mode, PPS, logs) and gets provisioned on first boot, as intended.
  - `PSRVELDRPOS` doesn't appear in the `CONFIG` reply. Check on the bench whether the UM982 accepts `CONFIG PSRVELDRPOS DISABLE` and whether it then shows in `CONFIG`. If it only lists non-default items, the fast-path text match must allow for that.
  - The module ships at 460800 on every port, so detection finds it on the first baud tried.

## Verification (bench, when hardware is available)
1. **Detection:** a UM980 and a UM982 on the same board (or two carriers) are each identified correctly. The UM980 path is unchanged: same commands and same `UNILOGLIST`.
2. **Provisioning:**
   - A factory-new UM982 is provisioned with no user action.
   - `CONFIG` shows `SIGNALGROUP 3 6` (`4 5` on a build below 10979), `HEADING VARIABLELENGTH`, COM1/2/3 at 460800 and no RXTYPE lock.
   - `UNILOGLIST` shows three logs.
   - After a power cycle, the fast path passes.
3. **Rate:** BESTNAVB and UNIHEADINGB both arrive at 20 ± 1 Hz on SG 3 6 (build ≥ 10979). If not, try SG 4 5 and record what changes.
4. **Static heading:** with antennas on a bar at a known azimuth (surveyed points or a known building edge), the heading matches within the receiver's σ, and `baseline_yaw_b` is checked.
5. **Drive test:**
   - Log UNIHEADING, BESTNAV and the INS yaw. Check:
     - INS yaw tracks the GNSS heading.
     - At standstill and when reversing, yaw holds (no 180° drag).
     - After power-on, yaw is valid before the tractor moves.
   - Replay the logs with `external/INSLIB/tools/replay.c`, with and without `m.yaw`, and compare.
6. **Baseline gate:** loosen ANT2 or shadow it. The gate rejects the epochs, the diagnostics count them, and INS yaw coasts on the gyro.
7. **Open questions to settle:**
   - The position-type values that mean a fixed heading.
   - The heading latency against BESTNAV in the same epoch.
   - VARIABLELENGTH against TRACTOR behaviour.
   - The 5 V current of each module at 20 Hz positioning + heading (the datasheet's 600 mW is for 10 Hz).
   - The module's COM1 and PPS logic level (3.3 V expected).
   - Whether SG 3 6 reaches 20 Hz for BESTNAVB and UNIHEADINGB together.
