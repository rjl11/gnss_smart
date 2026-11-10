# GNSS: UM980 auto-configuration by the MCU

## Context
The user should only have to connect a UM980, new or previously used, and the Pico configures it. No PC tool and no manual steps. This plan covers the command set, the boot-time state machine that detects, checks and provisions the receiver, and how commands and RTCM share the one port.

Decisions fixed by the user:
- The sequence starts with `FRESET`, then a 15 s wait for the reboot, then `CONFIG SIGNALGROUP 2` (all constellations), then a wait for that reboot, and it ends with `SAVECONFIG`.
- COM1, COM2 and COM3 all run at **460800**.
- **No port is ever locked** (`CONFIG RXTYPE` is never sent), because of the risk of bricking the receiver.
- **The Pico uses only COM1.** It carries commands, binary logs out and RTCM corrections in. COM3 is the module's USB-C port, left free for a PC running UPrecise. Receiver firmware upgrades are done that way and are out of scope for the firmware. The COM2 pins are not connected. Only their baud rate is set.
- **Hardware:** Both receivers are **ready-made modules**, not bare LGA parts. Each module exposes: a USB-C port that is **COM3** (for a PC, through a CH340 bridge), COM1 RX/TX and COM2 RX/TX on pins, PPS, 5 V and GND. The mapping was found on the UM982 module ([gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md)) and is assumed for the UM980 module. The Pico's UART1 goes to the COM1 pins.
  - **Exception, Zero 2 W companion only** ([pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md)): COM2 is wired both ways to the Zero's UART. The Zero writes only CRC-checked RTCM into COM2 RX and never sends commands, so the Pico stays the only device that configures the receiver (a PC in a `gnss_tcp` session reaches COM1 only through the Pico, which re-checks the config afterwards). COM2 TX carries `GPRMC COM2 1` and `GPZDA COM2 1`, which the Pico enables after the `UNLOG`s, for the Zero's NTP server. Whether the Zero is fitted is a compile-time choice: the Zero-companion firmware is its own build (plain Pico 2, TinyUSB instead of BTstack), and only that build adds the two COM2 logs to the target table and to the fast-path check. RTCM then enters COM2, not COM1. COM2 stays `AUTO`; no `RXTYPE` lock.

Sources: `docs/Unicore Reference Commands Manual For N4 High Precision Products_V2_EN_R1.15.pdf` (section numbers below refer to it) and `docs/datasheets/UM980_Datasheet.pdf`.

Related: [rp2350-feasibility.md](rp2350-feasibility.md) (port layout), [rtcm-over-bluetooth.md](rtcm-over-bluetooth.md) (RTCM source), [inslib-nav-suite.md](inslib-nav-suite.md) (what the logs feed), [web-config-server.md](web-config-server.md) (status and the "Reconfigure GNSS" action), [gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md) (the UM982 dual-antenna variant reuses this state machine with per-model deltas).

## Facts from the manual
- **`FRESET`** clears the saved configuration, ephemerides and position, sets all ports to 115200 and restarts the receiver (§8.1).
- **`CONFIG SIGNALGROUP`** (§4.48):
  - It reboots the receiver only if the value changes, and it saves itself without `SAVECONFIG`. That is why the manual says to send it before the other settings.
  - The UM980 supports SG 1, 2, 8 and 10.
  - SG 2 covers GPS L1C/A L1C L2C L2P L5, BDS B1I B3I B1C B2a B2b, GLO G1 G2 G3, GAL E1 E5a E5b E6, QZSS L1C/A L1C L2C L5 L1S, NavIC L5 and SBAS.
  - Only SG 8 is documented for 50 Hz. 20 Hz is a standard output rate (§7, parameter `0.05`), but check on the bench that SG 2 really delivers it.
- **RTCM on the command port:**
  - In rover mode the receiver identifies RTCM input by itself (§3.1).
  - The default port type is `RXTYPE AUTO` (§4.3), so COM1 accepts commands and RTCM together.
  - `FRESET` returns a used receiver that has a locked port to AUTO.
- **No command checksum on the UM980:** `CONFIG CMDFORMAT` isn't listed for it (§4.12). Commands are plain ASCII ending in CRLF.
- **Read-only checks:**
  - `CONFIG` with no argument returns one `$CONFIG,<item>,<command>*cs` line per setting (§4.1).
  - `UNILOGLIST` lists the active logs (§7.5.101).
- **`VERSIONB`** (msg ID 37) gives the product type (18 = UM980), firmware build and serial number (§7.5.2). `MODE ROVER SURVEY` needs UM980 build ≥ 7923 (§3.4).
- **`SAVECONFIG`** saves the logs (except ONCE logs) and the port settings (§8.3).
- **Doppler position prediction** is on by default (§4.34). When pseudoranges are poor, the receiver then reports a predicted position as a fix with 0 satellites. For INSLIB that is a fake measurement, so turn it off.
- **Command acknowledgement:** the reply is `$command,<cmd>,response: OK*cs`. An error reply is `$command,<cmd>,response: PARSING FAILED GRAMMAR ERROR,<token>*cs`. Both were confirmed on a UM982 (same N4 firmware family, [gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md)); confirm them once on a UM980.

## Target configuration
| Item | Command | Why |
|---|---|---|
| Signals | `CONFIG SIGNALGROUP 2` | All constellations and frequencies |
| Rover mode | `MODE ROVER SURVEY` | The manual's mode for precision agriculture and survey. Set explicitly so a different firmware default doesn't matter |
| Doppler prediction | `CONFIG PSRVELDRPOS DISABLE` | No fake fixes into INSLIB, which does its own prediction |
| PPS | `CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0` | 1 Hz, rising edge, 100 ms wide. `ENABLE` outputs PPS only once time has converged, so there are no edges before time is valid (§4.32) |
| Port locks | none: `CONFIG RXTYPE` is never sent | Risk of bricking. All ports stay at the default AUTO |
| Baud | `CONFIG COM2 460800`, `CONFIG COM3 460800`, `CONFIG COM1 460800` | COM1 is the Pico's port, so it goes last. A PC on the USB-C (COM3) or on the COM2 pins finds the receiver at a known rate |
| Logs | `UNLOG COM1`, `UNLOG COM2`, `UNLOG COM3`, then `BESTNAVB COM1 0.05`, `STADOPB COM1 1` | See below |

Logs:
- **BESTNAVB at 20 Hz** (§7.5.21) is the INSLIB input: position, velocity, per-axis σ, differential age, satellite counts and solution type.
- **STADOPB at 1 Hz** (§7.5.96) gives the HDOP for GGA.
- **UTC for NMEA** comes from the leap-second field in the binary header (§7.5.1).
- **Bandwidth:** BESTNAVB is about 24 + 128 + 4 B, so at 20 Hz about 3.1 kB/s. That is under 7 % of 460800.
- **Height:** BESTNAV reports height above MSL plus the undulation. INSLIB wants ellipsoidal height = hgt + undulation; convert in the parser.

## Boot state machine (core 0, module `gnss_um980_cfg`)
1. **Detect:**
   - Set UART1 to 460800, send `VERSIONB`, and wait up to 500 ms for a VERSION frame (sync `AA 44 B5`, msg ID 37).
   - If there is no reply, scan 115200, 921600, 230400, 57600, 38400, 19200 and 9600.
   - If no rate answers, report "no receiver" and try again every 5 s.
2. **Identify:** check that the type is 18 (UM980) and the build is ≥ 7923. Keep the serial number and firmware version for the diagnostics page.
3. **Check (fast path):**
   - Only when the receiver was found at 460800.
   - Send `CONFIG` and `UNILOGLIST` and compare the replies with the target table (text match on the `$CONFIG` lines and the log entries).
   - If everything matches, configuration is done: nothing is written to NVM, and boot takes about 1 s longer.
4. **Provision.** Runs on any mismatch, when the receiver was found at another baud, or from the "Reconfigure GNSS" action on the web page:
   1. `UNLOG` (quiets the current port), then `FRESET`. Switch UART1 to 115200.
   2. Wait **15 s**. Then send `VERSIONB` every 1 s until the receiver answers (error after 60 s).
   3. `CONFIG SIGNALGROUP 2` and wait for the OK. Wait **15 s**, then probe until the receiver answers; this works whether or not it rebooted. Send `CONFIG` and check that it shows `SIGNALGROUP 2`.
   4. `UNLOG COM1`, `UNLOG COM2`, `UNLOG COM3`, `MODE ROVER SURVEY`, `CONFIG PSRVELDRPOS DISABLE`, `CONFIG PPS ENABLE GPS POSITIVE 100000 1000 0 0`. Wait for each OK.
   5. `CONFIG COM2 460800`, `CONFIG COM3 460800`, and `CONFIG COM1 460800` last. The COM1 reply is expected at the old baud (verify on the bench). After it, switch UART1 to 460800, wait 100 ms and probe with `VERSIONB`.
   6. `BESTNAVB COM1 0.05`, `STADOPB COM1 1`.
   7. `SAVECONFIG` and wait for the OK.
   8. **Prove that it was saved:**
      - Send `RESET` and wait 15 s.
      - Detection must find the receiver at 460800, and the step 3 check must pass.
      - BESTNAVB must arrive at 20 ± 1 Hz.
5. **Run:** UART1 RX goes to the binary log parser. RTCM forwarding starts.

**Command helper:**
- Writes `cmd` + CRLF and waits up to 1 s for the matching `$command,…,response: OK`. Retries 3×, and logs failures.
- Binary logs and command replies share COM1, so the RX parser separates `$`/`#` ASCII lines from `AA 44 B5` binary frames.
- Every step's result goes to the diagnostics page: state, last error, receiver serial number, firmware.

**`FRESET` timing.** The user specified a fixed 15 s wait after `FRESET`. After that wait, the firmware probes until the receiver answers instead of assuming it is ready.

## UART1 TX: one port for commands and RTCM
UART1 TX has a single owner, the module `um980_tx`, fed by two queues:
- **RTCM queue:** whole, CRC-checked RTCM3 frames from the Bluetooth path ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)).
- **Command queue:** command lines from `gnss_um980_cfg` and, later, the web passthrough.

With the Zero 2 W companion, RTCM enters COM2 instead, and a third source appears: a serial-over-TCP session from a PC (`gnss_tcp` in [pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md)). It is a transparent byte stream, not command lines, so the arbiter doesn't queue it with the Pico's commands. While a session is open, the arbiter gives UART1 TX to it alone and holds the command queue.

Rules:
- Only whole RTCM frames or whole command lines are written, so the two never interleave inside a frame or a line.
- During provisioning, RTCM and serial-over-TCP bytes are dropped and counted. There is no fix yet, and stale corrections are useless.
- In "Run", commands go between RTCM frames.
- When a serial-over-TCP session closes, the Pico re-runs detection and the config check, because the PC may have changed anything.
- **Load:** RTCM at 1–3 kB/s is under 7 % of 460800. TX can use the FIFO or a DMA channel.
- No PIO state machine is needed for corrections.

Core split: everything here runs on core 0 ([rtos-decision.md](rtos-decision.md)). Core 1 gets GNSS samples only once state "Run" is reached. Until then INSLIB runs without GNSS. Core 1 holds `nav_suite_init()` until "Identify" has given the receiver model, but for at most 10 s after boot (start value, to be set from the module's measured boot time), so the filter also starts when no receiver answers ([gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md), "INSLIB integration").

## Verification (bench, when implemented)
1. A factory-new UM980 (or one reset with `FRESET` from UPrecise) is provisioned with no user action. The log shows every step OK and the total time (40–60 s, estimate, not yet measured).
2. Power-cycle: the fast path passes, no `FRESET` is sent, and BESTNAVB arrives at 20 Hz.
3. Change one setting by hand (e.g. `CONFIG COM1 115200` + `SAVECONFIG`). On the next boot it is detected and re-provisioned.
4. `CONFIG` shows SIGNALGROUP 2, COM1/2/3 at 460800, no RXTYPE lock, the PPS line and PSRVELDRPOS DISABLE. `UNILOGLIST` shows only the two logs (four with the Zero 2 W companion: plus `GPRMC` and `GPZDA` on COM2).
5. RTCM from SW Maps flows into COM1 while BESTNAVB streams:
   - The receiver reaches RTK fixed.
   - BESTNAVB stays at 20 Hz with no CRC errors.
   - A command sent during the RTCM stream still gets its OK.
6. A PPS edge arrives on the GPIO at 1 Hz after the fix, and none before.
7. **Open questions to settle on the bench:**
   - The acknowledgement and error text on a UM980 (confirmed on a UM982).
   - That the UM980 module has the same port mapping as the UM982 module (USB-C = COM3, pins = COM1 and COM2).
   - At which baud the `CONFIG COM1` reply arrives.
   - Whether SG 2 really gives 20 Hz. If not, fall back to SG 8 and record what that loses (GLONASS, among others).
   - Whether a factory-new UM980 is already at SG 2. If so, there is no reboot after `CONFIG SIGNALGROUP 2`.
   - Whether epochs with 0 satellites and an HDOP of 9999 appear in BESTNAVB / STADOPB, as the AiO v4.5 firmware sees in a UM982's GGA ([gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md), "Comparison"), and that the parser rejects them.

## Reference: a working receiver check
The AiO v4.5 autosteer firmware at https://github.com/renatorgr/Boards (`Firmware/v4.5_AiO_WEB/zRxCheck.ino`, `zReceiver.ino` and the README's "Receiver check", read 2026-11-10, GPL v3) does the same job for a UM982 and u-blox receivers, and it runs on real boards. It is a reference for behaviour, not code to copy.
- **Same flow:** search the baud rate, identify with `VERSIONA`, compare what the receiver has against a target list, write one command at a time, then `SAVECONFIG`. Its queries go between RTCM messages, never inside one, as our `um980_tx` arbiter does.
- **Its timeouts:** it waits 5 s for valid data at 460800, and 15 s for a silent receiver, before it searches the baud rate. That is a field figure to set beside our 10 s start value for core 1's wait for the receiver model.
- **A leading CR LF:** the V4.1 firmware it derives from (https://github.com/gunicsba/AgOpenGPS_Boards, `TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1/zGnss.ino`, commit `f57c395`, read 2026-11-10) sends every command as CR LF, the command, CR LF. The first CR LF ends whatever partial line or frame the receiver already holds. Our arbiter never interleaves, but the same prefix costs two bytes and protects the first command after boot or after dropped bytes; worth adding to the command helper.
- **Differences:** it never sends `FRESET` on its own (its console asks before `FRESET`, `UNLOG` and baud changes), and a receiver found at another baud stays there until the settings are loaded. Our sequence starts with `FRESET` by the user's decision.
