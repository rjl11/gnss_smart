# GNSS: UM98x (UM980 / UM982) auto-configuration and antenna mode

## Context
The user should only have to connect a receiver, new or previously used, and the Pico configures it. No PC tool and no manual steps. The receiver is a Unicore UM980 (one antenna) or a UM982 (two antennas, GNSS heading). Both run the same N4 firmware family and command set, have the same three UARTs and come on ready-made modules with the same connections, so one plan covers both. This plan covers the command set, the antenna-mode setting, the boot-time state machine that detects, checks and provisions the receiver, how commands and RTCM share the one port, and how the dual-antenna heading feeds INSLIB.

A single antenna gives only course over ground as a GNSS heading source (`automotive_mode`, [inslib-nav-suite.md](inslib-nav-suite.md)), and course over ground has three problems:
- It is undefined at standstill and below the minimum speed (2 m/s by default).
- It is wrong by 180° when reversing. INSLIB down-weights short reversing, but sustained reversing still drags yaw (comment on `ins_fuse_gnss_course_yaw()` in `external/INSLIB/src/ins.c`).
- After power-on, yaw stays unknown until the tractor drives forward.

A UM982 with both antennas outputs the heading of the ANT1 → ANT2 baseline. That is a real heading, valid at standstill and when reversing.

Decisions fixed by the user:
- **Antenna mode is a setting**, `single` or `dual`. It has a compile-time default, and the web config page can override it. A build whose default is single (UM980) can be switched to a UM982 with dual antennas. A change takes effect after a reboot. See "Antenna mode" below.
- **A UM982 with the slave antenna disabled is treated as a UM980**: the same logs and course-over-ground yaw, only with the UM982's lower maximum output rate. Its signal group in single mode was meant to be `CONFIG SIGNALGROUP 2 0`, but the UM982 refuses it (bench results 2026-11-12). The target table's fallback applies: keep `3 6`, so the slave antenna still tracks, and leave the heading unused.
- The sequence starts with `FRESET`, then a 15 s wait for the reboot, then `CONFIG SIGNALGROUP` (all constellations), then a wait for that reboot, and it ends with `SAVECONFIG`.
- COM1, COM2 and COM3 all run at **460800**.
- **No port is ever locked** (`CONFIG RXTYPE` is never sent), because of the risk of bricking the receiver.
- **The Pico uses only COM1.** It carries commands, binary logs out and RTCM corrections in. COM3 is the module's USB-C port, left free for a PC running UPrecise. Receiver firmware upgrades are done that way and are out of scope for the firmware. The COM2 pins are not connected. Only their baud rate is set.
  - **Exception, Zero 2 W companion only** ([pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md)): COM2 is wired both ways to the Zero's UART. The Zero writes only CRC-checked RTCM into COM2 RX and never sends commands, so the Pico stays the only device that configures the receiver (a PC in a `gnss_tcp` session reaches COM1 only through the Pico, which re-checks the config afterwards). COM2 TX carries `GPRMC COM2 1` and `GPZDA COM2 1`, which the Pico enables after the `UNLOG`s, for the Zero's NTP server. Whether the Zero is fitted is a compile-time choice: the Zero-companion firmware is its own build (plain Pico 2, TinyUSB instead of BTstack), and only that build adds the two COM2 logs to the target table and to the fast-path check. RTCM then enters COM2, not COM1. COM2 stays `AUTO`; no `RXTYPE` lock.
- **Dual-antenna heading mode is `CONFIG HEADING VARIABLELENGTH`**: the receiver does not hold the baseline length fixed.

Sources:
- `docs/Unicore Reference Commands Manual For N4 High Precision Products_V2_EN_R1.16.pdf` (Oct. 2026). Section numbers below refer to it. Against R1.15 (Jun. 2026, removed from `docs/`; still in git history), nothing this plan uses changed: the UNIHEADING, HWSTATUS, BESTNAV, PSRVELDRPOS and PPS sections and the position-type values read the same. Three log sections moved: STADOP §7.5.96 → §7.5.98, UNIHEADING §7.5.99 → §7.5.101, and UNILOGLIST §7.5.101 → §7.5.103.
- `docs/datasheets/UM980_Datasheet.pdf` and `docs/datasheets/UM980_User Manual_EN_R1.10.pdf` (UM980 hardware manual R1.10, 2026-06). Section and table numbers marked "UM980 HW" refer to the manual.
- `docs/datasheets/um982-user-manual-en-r1-1.pdf` (UM982 hardware manual R1.1, 2022-09). Section and table numbers marked "HW" refer to it.
- `docs/UM982_Product_Brief_En_mail.pdf` (UM982 product brief) and `docs/PRECISION PRODUCTS LIST 2026_En.pdf` (Unicore's 2026 product list). Marketing sheets: used only where they agree with or update the manuals.

Related: [rp2350-feasibility.md](rp2350-feasibility.md) (port layout), [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md) (RTCM source, NMEA out), [inslib-nav-suite.md](inslib-nav-suite.md) (what the logs feed), [web-config-server.md](web-config-server.md) (antenna-mode setting, status and the "Reconfigure GNSS" action), [gnss-mosaic-x5-vs-um980.md](gnss-mosaic-x5-vs-um980.md) (alternative receiver).

## Receivers
| | UM980 | UM982 |
|---|---|---|
| `VERSIONB` product type (§7.5.2) | 18 | 17 |
| Minimum build (`MODE ROVER`, §3.4) | 7923 | 7650 |
| Antennas | 1 | 2: ANT1 (master), ANT2 (slave) |
| Signal groups (§4.48) | 1, 2, 8, 10; one parameter only | two parameters (master, slave): `4 5` (default), `3 6`, `5 0`, `7 0`. Our build 18975 accepts exactly these (bench results 2026-11-12) |
| Max output rate | 50 Hz on SG 8 only (UM980 HW Table 1-1: "supports 50 Hz in specific mode"); 20 Hz standard | 20 Hz RTK + heading (HW Table 1-1, product brief); no 50 Hz option |
| GNSS heading | no | yes, on by default (§1.3) |
| Cold start (TTFF) | < 12 s (UM980 HW Table 1-1) | < 30 s (product brief) |
| Power (bare part, 3.3 V) | 145 mA typical, **180 mA maximum**, 480 mW (UM980 HW Table 2-3) | 180 mA typical, **300 mA maximum**, 600 mW (HW Table 2-4) |
| UART logic | LVTTL at VCC (3.0–3.6 V): in high ≥ 0.7 VCC, out high ≥ VCC − 0.45 V at 2 mA (UM980 HW Table 2-4) | LVTTL (product brief) |
| Firmware build | latest we have: R4.10Build25621; our module's build not yet read | R4.10Build18975, the latest we have and the one our module runs. See "Receiver firmware builds" |

This project uses 20 Hz on both, so the lower maximum rate of the UM982 costs nothing here.

## Antenna mode
- **Default:** `APP_GNSS_ANTENNA_MODE` in `firmware/app/app_config.h`, `single` unless the build says otherwise.
- **Override:** the web config page stores the mode in the flash config struct ([web-config-server.md](web-config-server.md)). "Revert to defaults" returns to the compile-time value.
- **When it applies:** it is read once at boot. A change needs a reboot, like the Wi-Fi settings. This keeps the target table and the filter's `automotive_mode` fixed while running.
- **Effective mode** from the setting and the receiver found at Identify:

| Setting | UM980 fitted | UM982 fitted |
|---|---|---|
| single | UM980 single | UM982 single (slave off) |
| dual | UM980 single; the diagnostics page warns "dual antenna set, UM980 fitted" | UM982 dual |

- **Switching single → dual:** fit the UM982 and ANT2, set `dual` and `baseline_yaw_b` on the web page, reboot. The fast-path check then fails on the signal group, the heading setting and the logs, and the receiver is re-provisioned. Dual → single works the same way. Swapping a UM980 for a UM982 with the setting left at single gives UM982 single, also after one re-provision.
- **Why not detection alone:** a UM982 can be fitted with one antenna, or with ANT2 on something that moves relative to the cab. The user decides whether the baseline is a heading; the firmware only refuses dual on a receiver that can't do it.

## Hardware
- **Ready-made modules, same connections.** Both receivers come on ready-made modules, not bare LGA parts. Each module exposes a USB-C port that is **COM3** (for a PC, through a CH340 bridge), COM1 RX/TX and COM2 RX/TX on pins, PPS, 5 V and GND. The mapping was found on the UM982 module (bench results below) and is assumed for the UM980 module; check it once. The Pico's UART1 goes to the COM1 pins. The wiring is identical for both, so swapping receivers is a plug change, not a board change. The UM982 module adds the ANT2 connector.
  - The bare UM982 (48-pin LGA) is not pin-compatible with the bare UM980 (54-pin LGA) (HW Table 1-1, §2.1), but the modules hide that. The pin-level rules in HW §3 (V_BCKP, BIF pull-ups, antenna feed, power-on ramp) are the module maker's job.
  - The modules have no RESET_N pin, so the firmware can't hard-reset a stuck receiver. It uses `RESET` over COM1, and power-cycling the 5 V would need a load switch (open question). If one is added, the bare part needs its supply below 0.4 V for **more than 500 ms** before the next power-on, with a monotonic ramp (UM980 HW §3.3). The module's own regulator and capacitors may stretch that, so measure the off time at the module's 3.3 V rail.
- **Power:** the module takes 5 V. The UM982's 600 mW figure is for 10 Hz PVT + 10 Hz RTK + 10 Hz heading, so at 20 Hz expect more. The module's regulator and the active antennas add to that, so measure the 5 V current of each module (estimate, not yet measured).
- **Logic level:** both bare parts have LVTTL UARTs at their 3.3 V supply (receivers table), which matches the Pico's 3.3 V GPIO. The module can still level-shift or buffer them, so check on the bench that the module's COM1 TX and PPS swing 3.3 V, not 5 V, before wiring them to Pico GPIO.
- **Antennas (dual mode):** ANT1 (master) and ANT2 (slave), both roof-mounted with a clear sky view.
  - A longer baseline gives a more accurate heading. The receiver doesn't fix the length (VARIABLELENGTH), but the mount must still be rigid. See "VARIABLELENGTH and the mount" below.
  - Record the baseline direction in the body frame (`baseline_yaw_b`, ANT1 → ANT2, measured from the body x-axis). Mounted across the cab (left → right), `baseline_yaw_b` is +90°.
  - `gnss_leverarm_b` in `ins_measurements_t` is ANT1's in every mode, because BESTNAV is the master antenna position.
  - ANT2 tracks fewer signals than ANT1. The HW manual (Table 1-1, 2022) lists BDS B1I/B2I/B3I, GPS L1 C/A/L2C, GLONASS L1/L2, Galileo E1/E5b, QZSS L1/L2. The newer product brief lists GPS L1C/A, L2C; BDS B1I, B3I (no B2I); GLONASS G1, G2; Galileo E1, E5b; QZSS L1C/A, L1C/B, L2C. Either way ANT2 has no L5/E5a. That matters for heading only; position comes from ANT1.

## Facts from the manual
- **`FRESET`** clears the saved configuration, ephemerides and position, sets all ports to 115200 and restarts the receiver (§8.1).
- **`CONFIG SIGNALGROUP`** (§4.48):
  - It reboots the receiver only if the value changes, and it saves itself without `SAVECONFIG`. That is why the manual says to send it before the other settings.
  - The first parameter is the master antenna's group, the second the slave's. **`0` disables the slave antenna** (Table 4-60). On a dual-antenna product an omitted second parameter is set to 0. A single-antenna product rejects a second parameter with an error.
  - UM980 SG 2 covers GPS L1C/A L1C L2C L2P L5, BDS B1I B3I B1C B2a B2b, GLO G1 G2 G3, GAL E1 E5a E5b E6, QZSS L1C/A L1C L2C L5 L1S, NavIC L5 and SBAS.
  - On the UM980 only SG 8 is documented for 50 Hz. 20 Hz is a standard output rate (§7, parameter `0.05`), but check on the bench that SG 2 really delivers it.
  - Table 4-61 lists only `4 5` (default) and `3 6` in its UM982 row. R1.16 adds a UMD982 row with `4 5`, `3 6`, `5 0` and `7 0`, the same four values our UM982 accepts. With **SG 3 6**, 20 Hz positioning needs build ≥ 10979, and then only "under specific log requests". `2 0`, the user's choice for UM982 single mode, isn't in the table, and the UM982 refuses it. Build 18975 accepts exactly `3 6`, `4 5`, `5 0` and `7 0` (bench results 2026-11-12).
  - Table 4-60 doesn't match what build 18975 tracks on the master in two places (bench results 2026-11-12, sky test): group 7 tracks no Galileo E6, though the table lists it, and group 3 tracks BDS B1C, not B2a, as footnote 6 says for builds before 23575.
  - The manual says an omitted second parameter becomes 0 on a dual-antenna product. On the UM982 that holds only for `5` and `7`, where `x 0` is valid anyway. Every other master-only form gets a grammar error, `4` included. Always send both parameters.
- **RTCM on the command port:**
  - In rover mode the receiver identifies RTCM input by itself (§3.1).
  - The default port type is `RXTYPE AUTO` (§4.3), so COM1 accepts commands and RTCM together. R1.16 lists `CONFIG RXTYPE` for the UM980 as well as the UM982. That changes nothing here: it is still never sent.
  - `FRESET` returns a used receiver that has a locked port to AUTO.
- **Command checksum:** `CONFIG CMDFORMAT` isn't listed for the UM980 (§4.12), but the UM982 lists `CONFIG CMDFORMAT 0` (bench results). Leave it at 0: commands are plain ASCII ending in CRLF.
- **Read-only checks:**
  - `CONFIG` with no argument returns one `$CONFIG,<item>,<command>*cs` line per setting (§4.1).
  - `UNILOGLIST` lists the active logs (§7.5.103).
- **`VERSIONB`** (msg ID 37) gives the product type, firmware build and serial number (§7.5.2). `MODE ROVER SURVEY` needs UM980 build ≥ 7923 or UM982 build ≥ 7650 (§3.4).
- **`SAVECONFIG`** saves the logs (except ONCE logs) and the port settings (§8.3).
- **Doppler position prediction** is on by default (§4.34). When pseudoranges are poor, the receiver then reports a predicted position as a fix with 0 satellites. For INSLIB that is a fake measurement, so turn it off.
- **`HWSTATUS`** (msg ID 218, §7.5.44, Tables 7-101 and 7-102) is the hardware status log. It is listed for the UM980, UM982 and UM981, among others, and supports **1 Hz output only**. The fields after the header are:
  - Field 2: an Int marked **"Reserved"**. Other firmware reads it as the chip temperature in thousandths of a °C (see "Reference: reading the receiver temperature" below), but the manual doesn't say so. The manual's own example has 66807 there.
  - DC09, DC10 and DC18 (float, V): core supplies with normal ranges of 0.85–1.0 V, 0.95–1.1 V and 1.7–1.9 V.
  - Clockflag (1 = ClockDrift valid), ClockDrift (equivalent velocity, m/s), a reserved float.
  - hwFlag (UCHAR): bit 0 oscillator/crystal, bit 1 VCXO/TCXO, bit 2 26/20 MHz, bit 4 internal/external clock, bit 7 check status valid. Then a reserved UCHAR, PLL_LOCK (USHORT) and two reserved UINTs.
- **Command acknowledgement:** the reply is `$command,<cmd>,response: OK*cs`. An error reply is `$command,<cmd>,response: PARSING FAILED GRAMMAR ERROR,<token>*cs`. Both were confirmed on a UM982 (bench results below); confirm them once on a UM980.
- **Heading (dual-antenna products):**
  - Heading is on by default (§1.3). It is the angle clockwise from true north to the baseline from the master antenna (ANT1) to the slave antenna (ANT2).
  - **`CONFIG HEADING`** (§4.18): modes `FIXLENGTH` (default), `VARIABLELENGTH`, `STATIC`, `LOWDYNAMIC`, `TRACTOR`. `LENGTH <cm> <tol cm>` sets a fixed baseline; it isn't used here. `RELIABILITY 1..4` sets the heading reliability threshold; the default is 3.
  - **`CONFIG HEADING OFFSET <heading> <pitch>`** (§4.19) adds an offset to the HEADING, GPTHS and HEADING2 outputs. The default is 0.
  - **`CONFIG SMOOTH HEADING <epochs>`** (§4.49) works on dual-antenna products only. It is not used, because the filter does its own smoothing and smoothing adds lag.
  - **`RESET`** (§8.2) has `RTK2` (slave antenna RTK) and `HEADING` options to restart the heading solution.
  - **`UNIHEADINGB`** (message ID 972, §7.5.101) contains the solution status and position type, the baseline length [m], heading 0–360° and pitch ±90°, heading σ and pitch σ [deg], the base station ID and satellite counts. The note under the table: if the receiver's own INS is enabled, `pos type` can be `INS`, and then the heading comes from the INS, not from the antennas.
  - **`HEADINGSTATUSB`** (ID 521, §7.5.43) echoes the `CONFIG HEADING LENGTH` setting. It isn't needed in "Run".
  - `BESTNAV` refers to the master antenna (ANT1); the slave antenna's position is in a separate log, `BESTNAVH` (§7.5.20).
  - Heading accuracy is **0.1° RMS for a 1 m baseline** (HW Table 1-1, and the product brief). It scales roughly with 1/baseline, so a 2 m bar gives about 0.05° (estimate, not yet measured). RTK accuracy is the same as the UM980: 0.8 cm + 1 ppm H, 1.5 cm + 1 ppm V.

## Receiver firmware builds
The latest builds we have are in `docs/firmware/unicore/`, with the original release notes and English translations in its [README](../docs/firmware/unicore/README.md). They come from https://github.com/AlksSAV/gnss-firmware (copied 2026-11-12). Flashing stays out of scope for the Pico firmware: UPrecise on a PC over USB-C (COM3), as in [rp2350-feasibility.md](rp2350-feasibility.md).

- **UM980 R4.10Build25621** (no date): adapts to the new BeiDou PRN numbering and updates the RTK library for it and for the BeiDou-2 retirement; fixes B2b-PPP after the PRN rework; steadier L6 MADOCA-PPP; Unicore-proprietary compact corrections; better high-reliability mode fixing; anti-spoofing detection with warning messages; QZSS L1C/B tracking and L1S (SLAS) positioning.
- **UM982 R4.10Build18975** (2025-04-30): TruePoint ionosphere service (`CONFIG TPIONUSE ENABLE`); QZSS L6 MADOCA-PPP (`CONFIG PPP ENABLE L6MDCPPP`) and its fusion in PPP auto mode; RTK-assisted PPP (cm within 20 s after an RTK fix, holds > 1 min after RTK loss); a configurable PVT reliability threshold; smoother best position when the position type changes; anti-jamming changes on the master antenna; GSV's last field is now the signal ID (NMEA 0183 v4.11 errata).

What this means here:
- **BeiDou PRN rework.** The UM980 notes say builds before 25621 lost B2b-PPP and RTK stability after BeiDou's PRN change. Update every UM980 module to 25621 or later before field use. Build 18975 is the newest UM982 build we have and its notes don't mention the PRN rework, so look for a newer UM982 build with the same fix. Whether 18975 is affected in RTK use is not known (not yet checked).
- **Build check at boot.** Identify already reads the build from `VERSIONA` for the diagnostics page. Add a warning (log and diagnostics) when a UM980 reports a build below 25621. Don't refuse the receiver.
- **GSV signal ID** only matters if the receiver's own NMEA GSV is passed through to SW Maps. Our NMEA out is built by the Pico, so it has no effect now.
- **Not used:** PPP (B2b, MADOCA), TruePoint and the proprietary correction format. This project uses RTK from NTRIP RTCM. RTK-assisted PPP could bridge RTK outages, but it needs PPP enabled, which the target configuration doesn't do.
- **Diagnostics candidates:** the UM980 anti-spoofing warning (log name not known yet; look it up in the N4 manual R1.16) and the UM982 PVT reliability threshold (command not yet looked up).

## Target configuration
| Item | UM980 single | UM982 single | UM982 dual | Why |
|---|---|---|---|---|
| Signals | `CONFIG SIGNALGROUP 2` | `CONFIG SIGNALGROUP 3 6` on build ≥ 10979, else `4 5` (same as dual) | `CONFIG SIGNALGROUP 3 6` on build ≥ 10979 (as the module ships), else `4 5` | All constellations and frequencies. The UM982 refuses `2 0` (bench results 2026-11-12), so UM982 single keeps the dual group and leaves the heading unused (no `CONFIG HEADING`, no UNIHEADINGB). `5 0` and `7 0` are accepted and turn the slave off. On the sky, `7 0` tracks the most master signals (about 119, against about 110 on `3 6`) but no Galileo E6; its BESTNAVB rate isn't checked (bench results 2026-11-12, sky test). Confirm 20 Hz on the bench |
| Heading mode | — | not sent | `CONFIG HEADING VARIABLELENGTH` | See "VARIABLELENGTH and the mount" |
| Heading reliability, offset | — | — | not sent (default 3, offset 0) | The firmware applies `baseline_yaw_b` itself |
| Rover mode | `MODE ROVER SURVEY` | same | same | The manual's mode for precision agriculture and survey. Set explicitly so a different firmware default doesn't matter |
| Doppler prediction | `CONFIG PSRVELDRPOS DISABLE` | same | same | No fake fixes into INSLIB, which does its own prediction |
| PPS | `CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0` | same | same | 1 Hz, rising edge, 100 ms wide. `ENABLE` outputs PPS only once time has converged, so there are no edges before time is valid (§4.32) |
| Port locks | none: `CONFIG RXTYPE` is never sent | same | same | Risk of bricking. All ports stay at the default AUTO |
| Baud | `CONFIG COM2 460800`, `CONFIG COM3 460800`, `CONFIG COM1 460800` | same | same | COM1 is the Pico's port, so it goes last. A PC on the USB-C (COM3) or on the COM2 pins finds the receiver at a known rate |
| Logs | `UNLOG COM1`, `UNLOG COM2`, `UNLOG COM3`, then `BESTNAVB COM1 0.05`, `STADOPB COM1 1`, `HWSTATUSB COM1 1` | same | the same, plus `UNIHEADINGB COM1 0.05` | See below |

Logs:
- **BESTNAVB at 20 Hz** (§7.5.21) is the INSLIB input: position, velocity, per-axis σ, differential age, satellite counts and solution type.
- **STADOPB at 1 Hz** (§7.5.98) gives the HDOP for GGA.
- **UNIHEADINGB at 20 Hz** (§7.5.101), dual mode only: the heading for `m.yaw` (see "Dual-antenna heading" below).
- **HWSTATUSB at 1 Hz** (§7.5.44) is for diagnostics only; nothing from it goes to INSLIB. It gives the receiver temperature (unconfirmed field), the three core supply voltages, the clock-drift flag and PLL_LOCK.
  - **Parser rule:** read field 2 as a signed integer in thousandths of a °C. Accept −40 000 to 125 000 (−40 to 125 °C); outside that, report "not reported". Until the bench check below confirms the field, the diagnostics page labels it "temperature (unconfirmed)".
  - Flag DC09, DC10 or DC18 outside the manual's normal range on the diagnostics page.
  - No thermal warning threshold yet. Set one from field data.
  - **Why a saved 1 Hz log and not a poll:** the AiO v4.5 firmware sends a one-shot `HWSTATUSA` every 10 s instead (see the reference section below). A saved log uses the binary parser that already exists, comes back after `RESET`, and puts no command-queue traffic between RTCM frames.
- **UTC for NMEA** comes from the leap-second field in the binary header (§7.5.1).
- **Bandwidth:** BESTNAVB is about 24 + 128 + 4 B, so at 20 Hz about 3.1 kB/s, under 7 % of 460800. HWSTATUSB adds about 24 + 44 + 4 B per second. In dual mode UNIHEADINGB adds about 24 + 44 + 4 B at 20 Hz, about 1.4 kB/s, so COM1 TX carries about 4.5 kB/s in total, about 10 % of 460800.
- **Height:** BESTNAV reports height above MSL plus the undulation. INSLIB wants ellipsoidal height = hgt + undulation; convert in the parser.

## Boot state machine (core 0, module `gnss_um98x_cfg`)
1. **Detect:**
   - Set UART1 to 460800, send `VERSIONB`, and wait up to 500 ms for a VERSION frame (sync `AA 44 B5`, msg ID 37).
   - If there is no reply, scan 115200, 921600, 230400, 57600, 38400, 19200 and 9600.
   - If no rate answers, report "no receiver" and try again every 5 s.
2. **Identify:**
   - Accept type 18 (UM980, build ≥ 7923) or 17 (UM982, build ≥ 7650). Report any other type as "unsupported receiver".
   - Keep the model, serial number and firmware version for the diagnostics page.
   - Work out the effective mode from the antenna-mode setting and the model ("Antenna mode" above). It picks the target-table column, and core 0 reports it to core 1.
3. **Check (fast path):**
   - Only when the receiver was found at 460800.
   - Send `CONFIG` and `UNILOGLIST` and compare the replies with the effective mode's column (text match on the `$CONFIG` lines and the log entries). In UM982 dual the `CONFIG` reply must also contain the HEADING VARIABLELENGTH line.
   - `UNILOGLIST` must show three logs, four in UM982 dual (two more each with the Zero 2 W companion).
   - If everything matches, configuration is done: nothing is written to NVM, and boot takes about 1 s longer.
4. **Provision.** Runs on any mismatch (including a changed antenna mode or a swapped receiver), when the receiver was found at another baud, or from the "Reconfigure GNSS" action on the web page:
   1. `UNLOG` (quiets the current port), then `FRESET`. Switch UART1 to 115200.
   2. Wait **15 s**. Then send `VERSIONB` every 1 s until the receiver answers (error after 60 s).
   3. `CONFIG SIGNALGROUP` with the mode's value, and wait for the OK. Wait **15 s**, then probe until the receiver answers; this works whether or not it rebooted. Send `CONFIG` and check that it shows the new signal group.
   4. `UNLOG COM1`, `UNLOG COM2`, `UNLOG COM3`, `MODE ROVER SURVEY`, `CONFIG PSRVELDRPOS DISABLE`, `CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0`, and in UM982 dual `CONFIG HEADING VARIABLELENGTH`. Wait for each OK.
   5. `CONFIG COM2 460800`, `CONFIG COM3 460800`, and `CONFIG COM1 460800` last. The COM1 reply is expected at the old baud (verify on the bench). After it, switch UART1 to 460800, wait 100 ms and probe with `VERSIONB`.
   6. `BESTNAVB COM1 0.05`, `STADOPB COM1 1`, `HWSTATUSB COM1 1`, and in UM982 dual `UNIHEADINGB COM1 0.05`.
   7. `SAVECONFIG` and wait for the OK.
   8. **Prove that it was saved:**
      - Send `RESET` and wait 15 s.
      - Detection must find the receiver at 460800, and the step 3 check must pass.
      - BESTNAVB (and UNIHEADINGB in dual) must arrive at 20 ± 1 Hz.
5. **Run:** UART1 RX goes to the binary log parser. RTCM forwarding starts.

**Mismatch case (setting dual, UM980 fitted):** the UM980 single column is the target, so a UM980 that already matches it passes the fast path and is not re-provisioned on every boot. Only the diagnostics warning shows.

**Command helper:**
- Writes `cmd` + CRLF and waits up to 1 s for the matching `$command,…,response: OK`. Retries 3×, and logs failures. `SAVECONFIG` gets **10 s**: the AgOpenWeb-HAT daemon allows that long for it (see "Reference: UM98x replies and timings" below). Measure the real times on the bench and set both waits from them.
- Binary logs and command replies share COM1, so the RX parser separates `$`/`#` ASCII lines from `AA 44 B5` binary frames.
- Every step's result goes to the diagnostics page: state, last error, receiver model, effective antenna mode, serial number, firmware.

**`FRESET` timing.** The user specified a fixed 15 s wait after `FRESET`. After that wait, the firmware probes until the receiver answers instead of assuming it is ready.

## UART1 TX: one port for commands and RTCM
UART1 TX has a single owner, the module `um980_tx` (named after the `drivers/um980` driver; it serves both receivers), fed by two queues:
- **RTCM queue:** whole, CRC-checked RTCM3 frames from the Bluetooth path ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)).
- **Command queue:** command lines from `gnss_um98x_cfg` and, later, the web passthrough.

With the Zero 2 W companion, RTCM enters COM2 instead, and a third source appears: a serial-over-TCP session from a PC (`gnss_tcp` in [pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md)). It is a transparent byte stream, not command lines, so the arbiter doesn't queue it with the Pico's commands. While a session is open, the arbiter gives UART1 TX to it alone and holds the command queue.

Rules:
- Only whole RTCM frames or whole command lines are written, so the two never interleave inside a frame or a line.
- During provisioning, RTCM and serial-over-TCP bytes are dropped and counted. There is no fix yet, and stale corrections are useless.
- In "Run", commands go between RTCM frames.
- When a serial-over-TCP session closes, the Pico re-runs detection and the config check, because the PC may have changed anything.
- **Load:** RTCM at 1–3 kB/s is under 7 % of 460800. TX can use the FIFO or a DMA channel.
- No PIO state machine is needed for corrections.

## Dual-antenna heading (UM982 dual only)

### VARIABLELENGTH and the mount
In VARIABLELENGTH mode the receiver doesn't check the baseline length against a fixed value. That trades one integrity check for robustness when the length isn't known. But the INS treats the GNSS heading as the heading of the body, so the baseline must stay rigidly fixed to the IMU even though its length isn't configured.
- **Gate in the parser:** keep a running median of the reported baseline length. Mark the heading invalid when an epoch's length differs from it by more than a set tolerance (start at 5 cm, tune on the bench). A sudden change means a wrong fix or a moved antenna.
- If ANT2 is ever placed on something that moves relative to the cab (an implement, for example), its heading is not the tractor heading and must not be fed to the INS. Set the antenna mode to single in that case.
- The manual also lists `TRACTOR`, but its description is unclear in the extracted table. Compare it once on the bench against VARIABLELENGTH. That is an open question; VARIABLELENGTH stays the decision.

### Where the offset is stored
`baseline_yaw_b` is an install setting, stored in Pico flash and editable on the web config page ([web-config-server.md](web-config-server.md)), next to the antenna mode. It is not stored in the receiver, so swapping a receiver doesn't lose it.

### INSLIB integration
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
- **One queue:** UNIHEADINGB and BESTNAVB go to core 1 through the same core 0 → core 1 queue, as separate items, each with its own time of validity, so their arrival order doesn't matter. Core 1 only gets a heading that passed the checks below, and fills `m.yaw` from it.
- **Auto-init:** with `auto_init` true, the INS bootstraps yaw from the external yaw measurement (`ins.h`, auto-init comment). In dual mode the yaw is then known before the tractor moves.
- **Mag:** unchanged (not fed to the INS).
- What is lost with `automotive_mode` off: no heading aiding while the dual-antenna heading is invalid but the position is good (e.g. ANT2 shadowed). The gyro holds yaw over such gaps. Field logs should show how often that happens. If it does, revisit: feed course yaw from the firmware side as a second `m.yaw` source only when the UNIHEADING is invalid and the tractor is moving forward.

### Outputs
- **NMEA to SW Maps:** add `$GNHDT` (true heading, from the INS yaw) at 10 Hz, only once the INS heading is valid. SW Maps and other apps show it as the heading. GGA/RMC are unchanged.
- **`$PAOGI`** (when Ethernet returns, [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)): the only Ethernet sentence, in every mode. Its heading field is the INS yaw, so nothing changes in dual mode except that the yaw is valid earlier.
- **Diagnostics page:** the heading solution status and position type, the baseline length and its median, heading σ, the count of gated epochs, and `baseline_yaw_b`.

## INSLIB init and the antenna mode
`automotive_mode` (yaw from GNSS course over ground) is an `ins_options_t` setting, fixed at `nav_suite_init()`. It follows the antenna mode: true in single mode, false in dual mode. INSLIB fuses course yaw and external yaw independently, and in dual mode course yaw adds no information and still pulls the wrong way when reversing.

The antenna-mode setting is read from flash at boot, before core 1 starts, so core 1 calls `nav_suite_init()` at once with `automotive_mode` = (setting is single). It does not wait for the receiver. The only re-initialisation is the mismatch case: setting dual, but Identify finds a UM980. Core 0 then reports single mode, and core 1 re-initialises the filter with `automotive_mode` true (re-align, then navigate), as a config change does ([web-config-server.md](web-config-server.md)). With no receiver at all, the filter runs ATTITUDE_ONLY in either setting.

## Core split
Everything here runs on core 0 ([rtos-decision.md](rtos-decision.md)): detection, provisioning, the parsers (including UNIHEADINGB, the baseline gate and the offset) and the TX arbiter. Core 1 gets GNSS samples only once state "Run" is reached; until then INSLIB runs without GNSS. Core 1 adds nothing for the UM982 except reading `m.yaw` and taking `automotive_mode` from the antenna mode at init.

## Firmware status
Done in the first demo (`firmware/`, built, not yet run on hardware):
- `APP_GNSS_ANTENNA_MODE` in `firmware/app/app_config.h`, set from CMake with `-DGNSS_ANTENNA_MODE=single|dual` (default single).
- `firmware/drivers/um980/um980_cfg.c`: Identify accepts both receivers and picks the effective mode, the signal-group command and the target items; provisioning and the fast-path check follow the mode (`HEADING VARIABLELENGTH` and UNIHEADINGB only in dual). UM982 single uses the dual group (`3 6`, or `4 5` below build 10979), as in the target table.
- `firmware/drivers/um980/um980.c` decodes UNIHEADINGB; `firmware/app/main.c` applies the fixed-type check, the baseline-length median gate, `baseline_yaw_b` (`APP_BASELINE_YAW_DEG`) and the σ floor, and sends the heading to core 1 as its own queue item with its own time of validity and delay. Core 1 fills `m.yaw` from it.
- `firmware/app/nav_core1.c` starts the filter at once from the setting and re-initialises only in the mismatch case. `APP_MODEL_WAIT_MS` is gone.

Not done yet:
- The flash override from the web page (no web server yet). Until then the compile-time default is the setting.
- `HWSTATUSB COM1 1`: the demo still provisions and checks two logs (three in dual), not three (four).
- `$GNHDT` output (no Bluetooth in the demo) and the diagnostics page.

## Verification (bench, when implemented)
1. **Provisioning, per mode:** a factory-new UM980, a UM982 with setting single, and a UM982 with setting dual are each provisioned with no user action. The log shows every step OK and the total time (40–60 s, estimate, not yet measured).
2. **Power-cycle:** the fast path passes, no `FRESET` is sent, and BESTNAVB arrives at 20 Hz (UNIHEADINGB too in dual).
3. **Change one setting by hand** (e.g. `CONFIG COM1 115200` + `SAVECONFIG`). On the next boot it is detected and re-provisioned.
4. **`CONFIG` and `UNILOGLIST`:**
   - UM980 single: SIGNALGROUP 2, COM1/2/3 at 460800, no RXTYPE lock, the PPS line and PSRVELDRPOS DISABLE; three logs.
   - UM982 single: SIGNALGROUP 3 6 (4 5 below build 10979); no UNIHEADINGB; three logs; BESTNAVB at 20 Hz.
   - UM982 dual: SIGNALGROUP 3 6 (4 5 below build 10979), HEADING VARIABLELENGTH; four logs.
   - Two more logs each with the Zero 2 W companion (`GPRMC` and `GPZDA` on COM2).
5. **Mode switching:**
   - A build with default single, UM980 fitted: single mode, `automotive_mode` true.
   - Same build: swap to a UM982 with ANT2, set dual and `baseline_yaw_b` on the web page, reboot. The receiver is re-provisioned to dual, and heading reaches `m.yaw`.
   - Set single again and reboot: re-provisioned to UM982 single, no UNIHEADINGB.
   - Setting dual with a UM980 fitted: the warning shows, the UM980 single table is used, the filter re-initialises once with `automotive_mode` true, and the next boot passes the fast path (no provisioning loop).
6. **RTCM** from SW Maps flows into COM1 while the logs stream:
   - The receiver reaches RTK fixed.
   - BESTNAVB stays at 20 Hz with no CRC errors.
   - A command sent during the RTCM stream still gets its OK.
7. A PPS edge arrives on the GPIO at 1 Hz after the fix, and none before.
8. **Dual mode:**
   - Static heading: with antennas on a bar at a known azimuth (surveyed points or a known building edge), the heading matches within the receiver's σ, and `baseline_yaw_b` is checked.
   - Drive test: log UNIHEADING, BESTNAV and the INS yaw. INS yaw tracks the GNSS heading; at standstill and when reversing, yaw holds (no 180° drag); after power-on, yaw is valid before the tractor moves. Replay the logs with `external/INSLIB/tools/replay.c`, with and without `m.yaw`, and compare.
   - Baseline gate: loosen ANT2 or shadow it. The gate rejects the epochs, the diagnostics count them, and INS yaw coasts on the gyro.
9. **Open questions to settle on the bench:**
   - The acknowledgement and error text on a UM980 (confirmed on a UM982).
   - That the UM980 module has the same port mapping as the UM982 module (USB-C = COM3, pins = COM1 and COM2).
   - The UM980 module's firmware build (`VERSIONA COM3`). If it is below 25621, flash `docs/firmware/unicore/um980/UM980_R4.10Build25621.pkg` with UPrecise and record the build before and after.
   - At which baud the `CONFIG COM1` reply arrives.
   - Whether UM980 SG 2 really gives 20 Hz. If not, fall back to SG 8 and record what that loses (GLONASS, among others).
   - ~~Whether the UM982 accepts `SIGNALGROUP 2 0`.~~ It doesn't (bench results 2026-11-12), so the target table's fallback is used.
   - Whether `7 0` would serve UM982 single mode better than `3 6`. Tracking is settled (bench results 2026-11-12, sky test): `7 0` gains BDS B2a and B2b and loses Galileo E6. Still open: its BESTNAVB rate, and which of those signals the NTRIP base sends (E6 only helps RTK if the base sends it).
   - Which signals ANT2 tracks in groups 6 and 5 (the sky test had no slave antenna). Group 6 is expected to lack GLONASS, which matters for heading.
   - Whether `CONFIG SBAS ENABLE` makes the receiver track EGNOS. No SBAS was tracked in any group.
   - Whether SG 3 6 reaches 20 Hz for BESTNAVB and UNIHEADINGB together. If not, try SG 4 5 and record what changes.
   - Whether a factory-new receiver is already at the target signal group. If so, there is no reboot after `CONFIG SIGNALGROUP`.
   - Whether the UM982 accepts `CONFIG PSRVELDRPOS DISABLE` and whether it then shows in `CONFIG` (it doesn't appear as shipped). If `CONFIG` only lists non-default items, the fast-path text match must allow for that.
   - Whether epochs with 0 satellites and an HDOP of 9999 appear in BESTNAVB / STADOPB, as the AiO v4.5 firmware sees in a UM982's GGA ("Comparison: the AgOpenGPS UM982 settings"), and that the parser rejects them.
   - Whether HWSTATUS field 2 really is the temperature in thousandths of a °C. Read it right after a cold power-on and again after 30 min of running; it should rise. Compare it with a thermometer on the module shield. Check the UM980 as well as the UM982. On the UM982 a read-only first check can be done now with the module on the PC: send `HWSTATUSA COM3` once and look for a plausible value.
   - The UNIHEADING position-type values that mean a fixed heading, and the heading latency against BESTNAV in the same epoch.
   - VARIABLELENGTH against TRACTOR behaviour, and what `CONFIG` shows after both are sent (the AgOpenGPS file sends both).
   - The 5 V current of each module at 20 Hz (the UM982 datasheet's 600 mW is for 10 Hz), and the module's COM1 and PPS logic level (3.3 V expected).

## Bench results (2026-11-06, UM982 module on a PC over USB-C)
The module was connected to a PC through its USB-C port (CH340 USB-serial bridge, Windows COM10). Only read-only queries and one-shot logs were sent, and nothing was saved. The antennas weren't connected or had no sky view: BESTNAV and UNIHEADING reported `INSUFFICIENT_OBS`, so heading, rate and fix tests are still open.

- **Identity:** `"UM982","R4.10Build18975"`, product type 17. Build 18975 is above every build limit in this plan (7650 for `MODE ROVER`, 10979 for 20 Hz on SG 3 6).
- **The USB-C port is COM3.** A one-shot `VERSIONA COM3` arrived on the USB port, and `VERSIONA COM1` / `VERSIONA COM2` didn't. The module's only active log is `KSXT COM3 0.1`, and that is what streams on the USB port. The user confirmed that the module's two UART pin pairs are **COM1 and COM2**. The port assignment in all plans follows from this: Pico on COM1, Zero 2 W companion on COM2, USB-C (COM3) left for a PC.
- **Command replies confirmed:**
  - OK: `$command,<cmd>,response: OK*cs`.
  - Error: `$command,<cmd>,response: PARSING FAILED GRAMMAR ERROR,<token>*cs`, for example for `VERSIONA COM1 ONCE`. A one-shot log is requested as `<LOG>A <port>` with no trigger word.
- **`CONFIG CMDFORMAT 0`** is listed on the UM982, although the manual doesn't list the command for the UM980. Leave it at 0 (no checksum).
- **As the module ships** (`CONFIG` and `MODE` replies):

| Item | As shipped | Target (UM982 dual) |
|---|---|---|
| Signals | `SIGNALGROUP 3 6` | `3 6` on build ≥ 10979 |
| Heading | `HEADING VARIABLELENGTH`, `HEADING RELIABILITY 3` | already matches |
| Mode | `MODE ROVER UAV DEFAULT` | `MODE ROVER SURVEY` |
| PPS | `PPS DISABLE` | `PPS ENABLE GPS POSITIVE 100000 1000 0 0` |
| Ports | COM1, COM2, COM3 at 460800 | already matches |
| Logs | `KSXT COM3 0.1` | `UNLOG`, then BESTNAVB, STADOPB, HWSTATUSB, UNIHEADINGB on the Pico's port |
| Other | `ANTENNA POWERON`, `SBAS DISABLE`, `PPP ENABLE E6-HAS`, `RTK TIMEOUT 60`, `ANTIJAM AUTO` | not touched by this plan |

- **Consequences:**
  - A new module fails the fast-path check (mode, PPS, logs) and gets provisioned on first boot, as intended. In single mode the signal group differs too.
  - `PSRVELDRPOS` doesn't appear in the `CONFIG` reply (open question above).
  - The module ships at 460800 on every port, so detection finds it on the first baud tried.

## Bench results (2026-11-12, UM982 signal-group sweep)
Same module, build 18975, on COM10 through the USB-C port (COM3). At the start it was at `SIGNALGROUP 4 5` with all ports at 115200, not the shipped `3 6` and 460800, so it had been changed since 2026-11-06. Every form of `CONFIG SIGNALGROUP` was sent: master only 0–11, and every pair 0–11 × 0–11 (156 commands). After each OK the script waited for the reboot, probed with `VERSION` and read `CONFIG`. At the end `4 5` was restored. No antenna had a sky view, so rate and tracking weren't checked.

| Command | Reply | `CONFIG` shows | Back (s after OK) |
|---|---|---|---|
| `3 6` | OK | `3 6` | ~10 |
| `4 5` | OK | `4 5` | ~10 |
| `5 0` | OK | `5 0` | ~10 |
| `7 0` | OK | `7 0` | ~9 |
| `5` | OK | `5 0` | ~10 |
| `7` | OK | `7 0` | ~9 |
| all other 150 | `PARSING FAILED GRAMMAR ERROR,` | unchanged | — |

- **`2 0` and `2` are refused.** UM982 single mode uses the target table's fallback.
- **Groups 5 and 7 are accepted on the UM982**, although Table 4-61 lists them only for other products. Both turn the slave antenna off.
- **The second parameter is required**, except for `5` and `7`.
- **The reboot takes about 10 s** from the OK until `VERSION` answers again: a 3 s pause, then 6–7 s of probing. The 15 s wait in the boot sequence is enough.
- Script: a Python/pyserial loop over the commands. It sends each one, waits up to 3 s for the `$command` reply, and after an OK probes `VERSION` once a second (60 s limit) before reading `CONFIG`.

## Bench results (2026-11-12, UM982 signal-group sky test)
Same module, build 18975, on miniPi over the USB-C port (COM3, `/dev/ttyUSB0`, 115200). Only the master antenna (ANT1) was connected, with a clear sky view, so these are master-antenna figures only. Each accepted group was measured twice, in the order `3 6`, `4 5`, `5 0`, `7 0`, `7 0`, `5 0`, `4 5`, `3 6` (19:22–19:52 local time), so the change in the sky over the run roughly evens out. For each run the script set the group, waited for the reboot, waited 150 s for tracking to settle, then decoded 60 epochs of `OBSVMB COM3 1` (1 Hz). It counted each satellite × signal with a measurement, and how many of those had carrier-phase lock (tracking status bit 10). Signal type and system come from the tracking status word (Table 7-182). At the end the script sent `FRESET`.

| Group | Satellites | Signals (run 1 / run 2) | Phase-locked |
|---|---|---|---|
| `7 0` | 40 / 41 | **117 / 121** | 111 / 114 |
| `3 6` | 40 / 38 | 112 / 107 | 109 / 102 |
| `4 5` | 40 / 37 | 96 / 89 | 92 / 86 |
| `5 0` | 40 / 37 | 79 / 75 | 73 / 72 |

Signals per epoch, mean of both runs (C/N0 was 35–45 dB-Hz throughout and doesn't separate the groups):

| Signal | `3 6` | `4 5` | `5 0` | `7 0` |
|---|---|---|---|---|
| GPS L1C/A | 10.5 | 10.5 | 10.9 | 11.0 |
| GPS L2C / L2P | 8.5 / 2.0 | 8.5 / 2.0 | 8.9 / 2.0 | 8.8 / 2.0 |
| GPS L5 | 6.0 | 6.0 | — | 6.5 |
| GLO G1 / G2 | 8.0 / 7.0 | 7.8 / 7.2 | 8.0 / 7.2 | 8.0 / 6.7 |
| GAL E1 / E5b | 9.9 / 10.0 | 9.5 / 9.5 | 9.0 / 9.0 | 11.0 / 11.0 |
| GAL E5a | 10.0 | 9.5 | — | 11.0 |
| GAL E6 | 9.0 | — | — | — |
| BDS B1I / B3I | 10.5 / 10.5 | 10.5 / 10.5 | 10.5 / 10.2 | 10.5 / 10.5 |
| BDS B2I | — | 1.0 | 1.0 | 1.0 |
| BDS B1C | 7.5 | — | — | 7.0 |
| BDS B2a / B2b | — | — | — | 7.0 / 7.0 |
| SBAS, QZSS, NavIC | — | — | — | — |

- **Every group sees the same satellites.** The difference between them is signals per satellite.
- **`7 0` tracks the most signals** but **no Galileo E6**, against Table 4-60. It is the only group with BDS B2a and B2b.
- **`3 6` is the only group with Galileo E6.** Its BDS third frequency is B1C, not B2a (footnote 6, build < 23575).
- **`5 0` drops GPS L5 and Galileo E5a**, so it is a dual-frequency group.
- **No SBAS in any group.** `CONFIG` listed no SBAS item, and `CONFIG SBAS ENABLE` wasn't tried.
- **After `FRESET`:** `SIGNALGROUP 4 5`, every port at 115200, `MODE ROVER UAV`. A plain `MODE ROVER` also reads back `MODE ROVER UAV`, so the target table's explicit `MODE ROVER SURVEY` is needed to get the survey profile.
- Script: Python with `termios` (no pyserial on miniPi). It parses OBSVMB as 40-byte records after a 4-byte count (§7.5.67) and checks the CRC-32 of each frame.

## Reference: UM98x replies and timings
From the AgOpenWeb-HAT daemon (https://github.com/chriskinal/AgOpenWeb-HAT, `core/src/um98x.cpp` and `core/include/aow/um98x.h`, commit `e4bd9ef`, read 2026-11-10; ported from the AiO New Dawn firmware, GPL v3). It reads and writes UM981/UM982 settings on real receivers. These are facts from its code, to confirm on our module; no code is taken.
- **Reply formats** the fast-path text match has to handle:
  - `CONFIG`: the `$command,…,response: OK` line, then one `$CONFIG,…` line per setting. The `$CONFIG,COM3,…` line is usually the last. The code ends the reply after it, or after 300 ms of silence.
  - `MODE`: one `#MODE,…;…*` line.
  - `UNILOGLIST`: lines that start with `<`. The first is the count (for example `<   3`), followed by that many log lines.
- **Timeouts:** 5 s per command and 10 s for `SAVECONFIG`. Ours were 1 s for everything.
- **Quiet read:** it sends `UNLOGALL COM1`, `UNLOGALL COM2`, `UNLOGALL COM3` before reading `CONFIG`, because the log stream gets in the way, and it notes that the read stops the GNSS stream for a few seconds. Our fast path reads `CONFIG` with BESTNAVB running, so check on the bench that the binary frames and the `$CONFIG` lines separate cleanly, or quiet the port first.
- **`UNLOGALL` versus `UNLOG`:** it uses `UNLOGALL <port>`; our sequence uses `UNLOG <port>`. Check in the manual and on the bench which one clears every log on a port.
- **Logs on before `SAVECONFIG`:** it restores the logs before saving, so the saved settings include them. Our provisioning does the same (logs in step 6, `SAVECONFIG` in step 7).

The original of that code is the AiO v26 ("New Dawn") Teensy firmware (https://github.com/AgOpenGPS-Official/Firmware_Teensy_AiO_26, `lib/aio_system/UM98xManager.cpp`, commit `2551bff`, read 2026-11-10, GPL v3). It adds three details; again facts from its code, and no code is taken:
- **`MODE` is the only query it retries:** up to 3 times, 100 ms apart. The other queries are sent once. That suggests the `MODE` reply can get lost. Our command helper retries every command 3×, which covers it.
- **An example `MODE` reply** from its comments: `#MODE,76,GPS,FINE,2382,246983200,0,0,18,47;MODE ROVER SURVEY,*2A`. The setting sits between the `;` and the `,*`.
- **During a read or write** it stops its own GNSS parser, and it caps a reply line at 512 characters.

### The AiO v26 default set for a UM981
The same firmware's web page loads a default command set for a UM981, a single-antenna receiver with its own INS (`lib/aio_system/web_pages/SimpleUM98xConfigPage.h`). Beside our target table:

| Item | AiO v26 UM981 default | This plan |
|---|---|---|
| Mode | `MODE ROVER SURVEY` | same |
| Baud | `CONFIG COM1 460800`, `COM2 460800`, `COM3 460800` | same |
| PPS | `CONFIG PPS ENABLE GPS POSITIVE 500000 1000 0 0` (500 ms wide) | same command, 100 ms wide |
| Signals | `CONFIG SIGNALGROUP 1` | `CONFIG SIGNALGROUP 2` |
| Doppler prediction | not set | `CONFIG PSRVELDRPOS DISABLE` |
| Log | `INSPVAXA COM3 0.1` (the receiver's INS solution, 10 Hz) | `BESTNAVB COM1 0.05`, `STADOPB COM1 1` |
| RTK | `CONFIG RTK TIMEOUT 120`, `CONFIG RTK RELIABILITY 3 1`, `CONFIG DGPS TIMEOUT 300` | not touched |
| Other | `CONFIG BESTPARA 16.00 10.00`, `CONFIG RTCMB1CB2A ENABLE`, `CONFIG ANTIJAM AUTO`, `CONFIG AGNSS DISABLE`, `CONFIG NMEAVERSION V410`, `CONFIG ANTENNA POWERON`, `CONFIG ANTENNADELTAHEN 0.0000 0.0000 0.0000` | not touched |

Its `CONFIG INS …`, `CONFIG IMUTOANT …`, `CONFIG INSSOL …` and `CONFIG INSDIRECTION AUTO` lines are for the UM981's own INS and don't apply to a UM980 or UM982. The rate and the rover mode agree with ours, which is a second field source for 460800 on all three ports.

## Comparison: the AgOpenGPS UM982 settings
The AiO v4.5 autosteer firmware at https://github.com/renatorgr/Boards (read 2026-11-10, GPL v3) loads the AgOpenGPS file `ConfigUM982D.txt` into a UM982 (`Firmware/v4.5_AiO_WEB/zRxConfig.h`, `RXCFG_UM982D`, 27 commands, then `SAVECONFIG`). It is a field-used command set for the same receiver, so it is a useful check on our dual-mode column. It is not a measurement on our module, and it changes none of our decisions.

| Item | AgOpenGPS file | This plan (UM982 dual) |
|---|---|---|
| Signals | `CONFIG SIGNALGROUP 4 5` | `3 6` on build ≥ 10979 |
| Heading mode | `CONFIG HEADING TRACTOR`, then `CONFIG HEADING VARIABLELENGTH` | `VARIABLELENGTH` only |
| Heading reliability | `CONFIG HEADING RELIABILITY 3` | default (3), not sent |
| Heading offset | `CONFIG HEADING OFFSET 90.0 0.0` | not sent; the firmware applies `baseline_yaw_b` |
| Heading smoothing | `CONFIG SMOOTH HEADING 0` | not sent |
| Mode | `MODE ROVER SURVEY` | same |
| PPS | `CONFIG PPS ENABLE GPS POSITIVE 500000 1000 0 0` | same, with a 100 ms pulse |
| Ports | COM1, COM2, COM3 at 460800 | same |
| RTK | `CONFIG RTK TIMEOUT 600`, `CONFIG RTK RELIABILITY 3 1`, `CONFIG DGPS TIMEOUT 600` | not touched (module default `RTK TIMEOUT 60`) |
| Other | `CONFIG MMP ENABLE`, `CONFIG SBAS DISABLE`, `CONFIG ANTIJAM AUTO`, `CONFIG AGNSS DISABLE`, `CONFIG RTCMB1CB2A ENABLE`, `CONFIG SMOOTH RTKHEIGHT 0` | not touched |
| Output | NMEA `GPGGA`, `GPHPR` (and others) at 10 Hz | binary BESTNAVB, STADOPB, UNIHEADINGB at 20 Hz |

Points to take from it:
- **`TRACTOR` and `VARIABLELENGTH` are both sent**, `TRACTOR` first. The manual lists them as modes of one command, so the second probably replaces the first, but that is not confirmed. Check with `CONFIG` after sending both when comparing the two modes on the bench.
- **A second data point for `TRACTOR`:** Balázs Gunics' V4.1 firmware (https://github.com/gunicsba/AgOpenGPS_Boards, `TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1/zWebPage.h`, `HPR_SETUP`, commit `f57c395`, read 2026-11-10) sets a UM982 up with `CONFIG HEADING TRACTOR` alone, plus `CONFIG HEADING OFFSET 90.0 0.0`, `CONFIG HEADING RELIABILITY 3` and `MODE HEADING2 TRACTOR`. It uses the heading only when the `HPR` quality is 4 (fixed baseline) and drops it after 1 s without `HPR`, the same rule as our `m.yaw.is_valid`.
- **`CONFIG RTK TIMEOUT`** decides how long the receiver keeps an RTK solution after corrections stop. AgOpenGPS raises it to 600 s; the module ships with 60 s. INSLIB bridges outages itself, so the short default suits us, but it should be a deliberate entry in the target table once tested.
- **`CONFIG MMP ENABLE`** (multipath mitigation) is worth a bench comparison near the cab roof.
- **A quirk seen in the field:** a UM982 GGA can arrive with `00` satellites and HDOP `9999.0`, and that firmware holds the last good values for up to 2 s. Watch for the same epochs in BESTNAVB and STADOPB; our gating on solution status should already reject them.

## Reference: a working receiver check
The same AiO v4.5 firmware (`Firmware/v4.5_AiO_WEB/zRxCheck.ino`, `zReceiver.ino` and the README's "Receiver check", read 2026-11-10, GPL v3) does the same job for a UM982 and u-blox receivers, and it runs on real boards. It is a reference for behaviour, not code to copy.
- **Same flow:** search the baud rate, identify with `VERSIONA`, compare what the receiver has against a target list, write one command at a time, then `SAVECONFIG`. Its queries go between RTCM messages, never inside one, as our `um980_tx` arbiter does.
- **Its timeouts:** it waits 5 s for valid data at 460800, and 15 s for a silent receiver, before it searches the baud rate.
- **A leading CR LF:** the V4.1 firmware it derives from (https://github.com/gunicsba/AgOpenGPS_Boards, `TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1/zGnss.ino`, commit `f57c395`, read 2026-11-10) sends every command as CR LF, the command, CR LF. The first CR LF ends whatever partial line or frame the receiver already holds. Our arbiter never interleaves, but the same prefix costs two bytes and protects the first command after boot or after dropped bytes; worth adding to the command helper.
- **Differences:** it never sends `FRESET` on its own (its console asks before `FRESET`, `UNLOG` and baud changes), and a receiver found at another baud stays there until the settings are loaded. Our sequence starts with `FRESET` by the user's decision.

## Reference: reading the receiver temperature
The same AiO v4.5 firmware (`Firmware/v4.5_AiO_WEB/zReceiver.ino`, `zReceiver.h`, `zHandlers.ino` and the README, read 2026-11-12, GPL v3) shows a UM982's temperature on its status page. These are facts from its code and README, to confirm on our modules; no code is taken.
- **Command:** a one-shot `HWSTATUSA` every 10 s (`RX_TEMP_MS`). It is sent only when the port is otherwise quiet: data arrived in the last 2 s, no settings read or write is running, no console reply is pending and the TX queue is empty. Polling stops after 3 polls with no answer (`RX_TEMP_TRIES`). The `$command,HWSTATUSA,response: OK` reply is kept off its console.
- **Parsing:** it checks the line's CRC-32 and drops a damaged line. It reads the first field after `;` as thousandths of a °C and rounds it to whole degrees. A value outside −40 000 to 125 000 counts as "not reported". The page shows the reading as current for 30 s.
- **Example** from its README, a UM982 on R4.10Build13495 on the bench (2026-11-10): `#HWSTATUSA,71,GPS,FINE,2444,241114900,0,0,18,13;59493,0.914,1.025,1.805,1,-0.822,0.0,0x84,1,0x0377,0,0*6e81c92f`. It reads that as 59 °C. The three supplies (0.914, 1.025, 1.805 V) are inside the manual's normal ranges. Our module runs R4.10Build18975 and hasn't been checked.
- **Not confirmed:** the README says the temperature mapping is deduced, because the manual marks the field "Reserved". It proposes a cold-start against warm-start comparison and records no result. So this is their deduction, not a measured fact. Our bench check is in Verification 9.
- It sets no warning threshold for the receiver temperature and parses none of the other HWSTATUS fields.
