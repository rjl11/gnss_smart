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
- There is no UM982 datasheet in `docs/datasheets/` yet. Add `UM982_Datasheet.pdf` there before hardware design. Power, size, pinout and heading accuracy below are open until then.

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
- **Everything else in the UM980 plan holds for the UM982:** `FRESET`, `MODE ROVER SURVEY`, `CONFIG PSRVELDRPOS`, `CONFIG PPS`, `CONFIG COMx`, `BESTNAVB`, `STADOPB`, `UNILOGLIST`, `SAVECONFIG` and `RXTYPE AUTO`. `BESTNAV` refers to the master antenna (ANT1); the slave antenna's position is in a separate log, `BESTNAVH` (§7.5.20).

## Hardware
- **Antennas:** ANT1 (master) and ANT2 (slave), both roof-mounted with a clear sky view.
  - A longer baseline gives a more accurate heading. The receiver doesn't fix the length (VARIABLELENGTH), but the mount should still be rigid. See "VARIABLELENGTH and the mount" below.
  - Record the baseline direction in the body frame (`baseline_yaw_b`, ANT1 → ANT2, measured from the body x-axis) and the ANT1 lever arm.
  - Mounted across the cab (left → right), `baseline_yaw_b` is +90°.
- **Lever arm:** `gnss_leverarm_b` in `ins_options_t` is ANT1's, because BESTNAV is the master antenna position.
- **Ports: no change.**
  - UART1 ↔ COM1 at 460800 carries commands, binary logs out and RTCM in.
  - COM2/COM3 are set to 460800 and not connected. On the Zero 2 W companion, COM2 is used as described in [gnss-um980-config.md](gnss-um980-config.md).
  - No port is ever locked (`CONFIG RXTYPE` is never sent). PPS goes to the same GPIO.
- **Footprint:** whether the UM982 is pin-compatible with the UM980 (so one carrier board takes both) is **not yet verified**. Check it against the UM982 datasheet. If it isn't, the variant needs a second carrier board.
- **Power:** take the figure from the UM982 datasheet. It's expected to be above the UM980's 480 mW (estimate, not yet measured).

## Detection and configuration (core 0, `gnss_um980_cfg` extended)
The state machine in [gnss-um980-config.md](gnss-um980-config.md) stays as it is, except for these changes. Renaming the module to `gnss_unicore_cfg` is left to the implementation.

1. **Identify** accepts type 18 (UM980, build ≥ 7923) or 17 (UM982, build ≥ 7650). The model goes into the diagnostics and decides which target table applies. Any other type is reported as "unsupported receiver".
2. **Target table, per model:**

| Item | UM980 | UM982 |
|---|---|---|
| Signals | `CONFIG SIGNALGROUP 2` | `CONFIG SIGNALGROUP 4 5` (default). Fall back to `3 6` if 20 Hz isn't reached and the build is ≥ 10979 |
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
- **`m.yaw.stddev_rad`:** UNIHEADING's heading σ in radians, with a floor (start at 0.2°, tune from logs). Receiver σ is usually optimistic.
- **`m.yaw.is_valid`** is true only when all of these hold:
  - The solution status is SOL_COMPUTED.
  - The position type is a fixed-ambiguity type (e.g. `NARROW_INT`; confirm the values on the bench).
  - The position type is not `INS`.
  - The baseline-length gate passes.
  - The log is no older than INSLIB's 500 ms limit (`INS_MAX_DELAY_MS`).
- **`m.yaw_delay_ms` / `yaw_delay_us`:** the same as the GNSS epoch, from the log's GNSS time against the PPS-anchored capture time.
- **One epoch:** UNIHEADINGB and BESTNAVB with the same GNSS time go to core 1 in the same epoch, through the same core 0 → core 1 queue. Core 1 only gets a filled `m.yaw`.
- **`automotive_mode` off on a UM982.** INSLIB fuses the course over ground and the external yaw independently. On a UM982, course yaw adds no information and still pulls the wrong way when reversing. It is an `ins_options_t` setting, so it is fixed at `nav_suite_init()`. Core 1 therefore waits for the model from step 1 (Identify) before init. Identify comes within about a second of the receiver answering, well before the first fix.
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

## Verification (bench, when hardware is available)
1. **Detection:** a UM980 and a UM982 on the same board (or two carriers) are each identified correctly. The UM980 path is unchanged: same commands and same `UNILOGLIST`.
2. **Provisioning:**
   - A factory-new UM982 is provisioned with no user action.
   - `CONFIG` shows `SIGNALGROUP 4 5`, `HEADING VARIABLELENGTH`, COM1/2/3 at 460800 and no RXTYPE lock.
   - `UNILOGLIST` shows three logs.
   - After a power cycle, the fast path passes.
3. **Rate:** BESTNAVB and UNIHEADINGB both arrive at 20 ± 1 Hz on SG 4 5. If not, try SG 3 6 (build ≥ 10979) and record what changes.
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
   - Pin compatibility with the UM980 and power, from the UM982 datasheet.
   - Whether SG 4 5 reaches 20 Hz.
