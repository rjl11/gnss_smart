# IMU: TM210 auto-configuration by the MCU

## Context
The user should only have to connect a TM210, new or previously used, and the Pico configures it. No PC and no ImuAssistant. This plan covers:
- the EasyProtocol objects involved
- the target settings
- the boot-time state machine that detects, checks and provisions the IMU

It follows the same pattern as the UM980/UM982 ([gnss-um98x-config.md](gnss-um98x-config.md)). The resulting IMU data and its conversion for INSLIB are in [imu-tm210.md](imu-tm210.md).

SYD's documents disagree with the real TM210 in several places. **Where they differ, the bench results below (2026-11-06) win**, and the documents are quoted only for background.

Sources:
- **Bench tests on a TM210** (node ID 925) over its USB port, 2026-11-06.
  - Every write was RAM only (bit 18 = 0). The original settings were restored afterwards.
  - Scripts: `tm210_probe.py`, `tm210_odr_test.py`, `tm210_verify.py`, `tm210_bit27.py`. They lived in the session scratch space, not the repo; their results are recorded in this plan.
- `docs/TransducerM_TM3xx_UserGuide_EN_V135-R1.pdf`: EasyProtocol objects Request 12, Acknowledge 13, Setting 21, Status 22, Calibration 23, Raw 41. Also the sections "Communication Protocol" and "Change UART Baudrate".
- `docs/TransducerM_TM100-200_UserGuide_EN_V131.pdf`: the older models; no Setting object definition.
- `docs/datasheets/TransducerM_TM210_Datasheet_EN_V126-R.pdf`: ODR values, baud range, start-up times.
- `docs/TransducerM_AN002_EN_V11.pdf`: run-time forced calibration for AGVs.
- SYD Dynamics Q&A, https://www.syd-dynamics.com/questions-answers/ (read 2026-11-06): ODR vs Inhibit Time per model, baud recovery per model, the effect of the sensor enable checkboxes, calibration.
- SYD Dynamics example code in `external/syd_dynamics/` (vendored, not compiled; overview in its `README.md`):
  - `TransducerM_Example_CPP_QT_V131-R1/EasyProfile/EasyObjectDictionary.h` (EasyProfile V1-3-1-R1, 2025): `Ep_Settings`, `Ep_Switches` and `Ep_Combo` as SYD defines them.
  - `TransducerM_Example_CPP_QT_V131-R1/HelloMotionModule.cpp`, `Example_ChangeSetting()`: the vendor's read-modify-write sequence for settings.
  - `TransducerM_Example_RaspberryPiPico_V1-1/`, `TransducerM_Example_STM32_CPP/`, `TransducerM_Example_ROS-pkg_V1-9/`, `TransducerM_Example_ROS2-pkg_V1-1/`, `TransducerM_Protocol_C_V1-2-1-Beta/`: embedded, Linux and C builds of the same library.
    - None of them write settings.
    - The STM32 example receives with `HAL_UARTEx_ReceiveToIdle_DMA`, the same DMA-plus-idle pattern planned for UART0.
    - Its EasyProtocol writes the CRC byte by byte, which avoids unaligned access.

## Facts that shape the design
### Packet and addressing
- **Layout:** `AA 55 | len | 4-byte header | payload | CRC16 Modbus`. The CRC covers len + header + payload and is stored little-endian.
- **Header bits, LSB first:** CMD (7), 3 bits, fromId (11), toId (11).
  - The user guide and the QT library call the 3 bits "reserved, must be zero". The newer embedded builds (Pico, STM32, ROS) call them QoS.
  - **On the TM210 they read 0 on every packet**, even while the Status object reported QoS 4–5.
  - So the parser accepts any value and ignores it. The host sends 0. QoS comes from the Status object.
- **IDs:** host = 2, broadcast = 0, unspecified = 1 (`EP_ID_*` in `EasyObjectDictionary.h`).
  - Sensor node IDs are 0x064–0x7FF (100–2047) (user guide, "Overview of EasyProtocol").
  - Both the node ID and a device name of up to 12 characters can be changed in ImuAssistant ("TransducerM – Node ID, Firmware Version, UUID"). So the firmware takes the node ID from each reply, never a constant; `tm210.c` already does.
  - A reply to a Request is addressed to whoever sent it, so TO_ID = 2 for our host (user guide, "Request").

### Reading and writing settings (bench-verified)
- **Read:** send a Request (CMD 12, payload `{cmdRequest = 21, 0, 0, 0}`), broadcast is fine. The TM210 answers with its Setting object (CMD 21, 20-byte payload). The reply's `fromId` is the node ID.
- **Write:**
  1. Start from the read-back and change only the wanted fields. Set bit 17 (`request_acknowledge`) = 1 and bit 18 (`save_Setting_Permanently`) = 0.
  2. Send to `toId = <node ID>`, as the vendor's `Example_ChangeSetting()` does.
  3. The Acknowledge (CMD 13, `cmdAck = 21`) came back within 1.5 s every time. The new values were in effect straight away, with no restart: the output rate, output switches and USB output all changed on the spot.
     - **Only byte 0 of the Ack counts.** The user guide ("Acknowledge") calls bytes 1–3 "value undefined". On our TM210 they echoed switch bytes 1–3 of the Setting just written: Ack `15 a0 1a 00` for switches `5b a0 1a 00` (miniPi, 2026-11-12) and `5f a0 1a 00` (PC, 2026-11-13). `tm210.c` checks only byte 0.
- **Bits 17 and 18 are stored as sent**, and the read-back echoes them. A TM210 last saved from ImuAssistant reads bit 18 = 1. **The boot check ignores bits 17 and 18.**
- **Read-modify-write is required**, because several fields hold values the documents call fixed but this TM210 doesn't have (next section).

### `Ep_Settings` on the TM210 (20-byte payload)
| Bytes | Library name | On the TM210 | Handling |
|---|---|---|---|
| 0–3 | `switches` | bit fields, see below | |
| 4–5 | `reserved` ("always 1152") | read 0 on one occasion and 1152 on another | copied, not checked |
| 6–7 | `uart1_Baudrate` | unit 100 bps: 9216 = 921600, 1152 = 115200. A RAM write of 9216 was acknowledged and read back | target 9216 |
| 8–9 | `can_Baudrate` | read 2500. The TM210 has no CAN bus | copied, not checked |
| 10–11 | `gain_Acc` | ×0.01, fusion only (read 260 and 208) | copied |
| 12–13 | `gain_Mag` | ×0.01, fusion only | copied |
| 14–15 | `inhibitTime` | read 10. Changing the rate didn't touch it | copied, not checked |
| 16–17 | upper half of `silentTime` in the library; here a separate 16-bit field | read 1, probably Silent Time in seconds | target 0 |
| **18–19** | (part of `silentTime` in the library) | **output data rate in Hz, uint16.** Verified: 50 → 20,000 µs apart, 200 → 5,000 µs, 800 → 1,250 µs | target 800 |

The library declares `silentTime` as a uint32 at bytes 16–19. On the TM210, bytes 18–19 are really the ODR, so the firmware codec defines its own struct: two uint16 at 16 and 18.

### Switch bits on the TM210 (LSB first)
| Bit | Meaning | Bench result | Handling |
|---|---|---|---|
| 0–2 | gyro / acc / mag enable | fusion only; the Q&A says no effect on raw | copied, not checked |
| 3 | GyroErrFilter | fusion | copied |
| 4 | MagSelfAdaptFilter | fusion | copied |
| 5 | Status output | | 0 |
| 6 | Raw output (CMD 41) | | 1 |
| 7 | reserved1 | read 0 | copied |
| 8 | Quaternion output (CMD 32) | | 0 |
| 9 | reserved2 | read 0 | copied |
| 10 | Euler output (CMD 34) | | 0 |
| 11 | RPY output (CMD 35) | | 0 |
| 12 | Gravity output (CMD 36) | | 0 |
| 13 | reserved3 | read 1 | copied |
| 14–16 | reserved4 | read 0b010 | copied |
| 17 | request_acknowledge | stored and echoed | 1 on writes, ignored in the check |
| 18 | save_Setting_Permanently | stored and echoed | 0 on the RAM write, 1 on the commit, ignored in the check |
| 19 | UART1 output | | 1 |
| **20** | **USB output** (the library calls it reserved5, "must be 0") | set: USB streams; clear: USB goes silent | **1** |
| 21 | `enable_USB1_CAN1` | read 0 while USB streamed; the TM210 has no CAN bus | copied, not checked |
| 22 | reserved6 | read 0 | copied |
| 23 | temperature calibration ("set to 0 is recommended") | | 0 |
| 24–25 | bootMode: 0 Auto, 1 Static, 2 Dynamic | | 0 |
| **26** | **CMD 42 output**, an object not in any SYD document | set: CMD 42 at the output rate | **0** |
| **27** | **Combo output (CMD 43)** | set: CMD 43 at the output rate | **0** |
| **28** | unknown | set: all output dropped to about 6 Hz (160 ms apart) | **0** |
| 29–31 | not tested | read 0 | copied |

The library calls bits 26–31 "unused, must be 0", but bits 26–28 clearly do something.

### Other constraints
- **Flash or RAM:**
  - Bit 18 = 0 applies the settings in RAM only. The guide calls this "safer than programming the flash memory", and flash needs a stable supply.
  - A power cycle reloads the flash copy. Boot mode only takes effect after a power cycle, and only if saved to flash.
- **Output rate (ODR):** the TM210 offers 800, 400, 200, 100, 50, 25, 10, 5 and 1 Hz (datasheet). The SYD Q&A confirms it is an ODR model, with no Inhibit Time.
- **Baud:**
  - The factory default is 115200. Supported rates are 1M, 921600, 460800, 230400, 115200, 57600, 38400, 9600, 2400 and 1200.
  - The datasheet recommends ≥921600 for ODR ≥ 400 Hz. The Q&A asks for ≥230400 or ≥460800 at 400/800 Hz. 921600 with raw output only is well inside both.
  - **A new baud only takes effect from flash, after a power cycle** (bench, 2026-11-13, Pico UART0). Written to RAM, it is acknowledged at the old baud and reads back as the new value, but the UART keeps running at the old baud: nothing answered at 921600 right after the write or 2 s later, and the next scan found it at 115200 again. Saved to flash (bit 18 = 1) and power-cycled, the TM210 came up at 921600 with 802 Hz raw output and no CRC errors.
  - At 115200 an 800 Hz raw stream is limited by the line: about 201 packets/s arrive, against about 235 that would fit.
  - **Recovery when the baud is unknown:** for the TM210 the vendor's answer is the USB port ("simply use its USB interface to connect and find or reset the baudrate"). The 6-second 115200 SCAN lock in the user guides applies to the TM200/TM35x only. The firmware scans the UART baud rates itself, and USB (bit 20) is the manual fallback.
- **USB:** the USB port is a virtual COM port (VID:PID 0483:5740). Its baud setting has no effect. With bit 20 on, it carries the same objects at the same rate as the UART: 800 Hz raw-only ran over USB with no gaps.
  - **With USB connected, the TM210 sometimes ignores requests on its UART** (bench, 2026-11-13). After a power cycle with USB plugged in to dietpi, it kept streaming on the UART but answered no Setting request from the Pico at any baud, in two different settings states. Unplugging USB fixed it at once. Earlier the same day it did answer on the UART with USB connected, so the trigger is not known. For UART work, unplug USB.
  - Right after it enumerates, the USB port may not answer for a few seconds: the first two Setting requests went unanswered, the third worked.
  - The TM210 runs from the Pico's VBUS alone (5 V, 80 mA).
- **Calibration commands** (CMD 23: CalibB, AN002 forced calibration, heading fix, compass calibration) affect only the TM210's own fused outputs. **We use raw data only, so the firmware never sends them.** INSLIB estimates the biases itself.
  - The user guide gives more reasons not to send them. A power loss during CalibB's 20 s flash write "may result in unrecoverable damage" ("Boot Mode"), and its success report (Calibration object type 0x14) asks for a power cycle afterwards ("Calibration").
  - The TM210 also sends Calibration objects (CMD 23) back as status reports, but only after a calibration command, so the parser never expects them.
- **Status object (CMD 22):** answered on request while Status output was off (17.8 °C, internal rate 802 Hz, QoS 5).
- **`Ep_Combo` (CMD 43)** is a packed set of integers: gyro in 1e-5 rad/s, acc in 1e-5 g with the library's 1 g = 9.794 m/s², plus QoS, temperature and rate. It is not used. Raw (CMD 41, float32) keeps full resolution and is what [imu-tm210.md](imu-tm210.md) is built on.
- **Start-up:** Auto boot gives a 3.2 s cold start, and Static boot 10–30 s. The TM210 answers requests even during its silent time.
  - The TM3xx user guide ("Boot Mode", "Opening the Device") gives longer figures. Static boot needs 30 s of stillness with silent time 0, or silent time + 10 s. In Static or Auto mode the module "may ignore" commands for 13.5–30 s after power-on.
  - These are the guide's figures, not measured on our TM210, which answered 3.2 s after an Auto cold start.
  - Detect (step 1 below) retries every 2 s and never gives up, so a slow first answer after a Static boot is expected, not a fault.

### Cross-check: the TM171 in the AiO v4.5 firmware
An independent source for the same protocol: the AiO v4.5 autosteer firmware at https://github.com/renatorgr/Boards (`Firmware/v4.5_AiO_WEB/TM171.ino` and its README, read 2026-11-10, GPL v3). It reads and writes the Setting object of a **TM171**, checked on a real unit on 2026-10-24. These are their results on a TM171, not measurements on our TM210. Its code is GPL v3, so we take the facts, not the code.

- **The layout matches ours:** baud at bytes 6–7 (×100 bps), inhibit time at 14–15 (read 10), **output rate in Hz at bytes 18–19**, switch bit 13 = 1 and bits 14–16 = 2. Their first guess also followed SYD's documents and was wrong in the same places.
- **Writes need the node ID:** their TM171 answered requests sent to ID 0 but ignored every settings write sent that way. That agrees with writing to `toId = <node ID>`.
- **Bytes 16–17: the two firmwares disagree.** Their unit read 0 there, and the renatorgr firmware concluded the TM171 has no Silent Time at that place. The firmware it was ported to, Balázs Gunics' V4.1 (https://github.com/gunicsba/AgOpenGPS_Boards, `TeensyModules/V4.1/Firmware/Autosteer_gps_teensy_v4_1/TM171.ino`, commit `f57c395`, read 2026-11-10), reads the same bytes as the **power-on silent time in seconds** (0–65, the lower half of the guide's 32-bit word) and writes it from its web page. That is our reading too, and ours read 1. "Still to do" item 7 settles it for the TM210.
- **Calibration commands:** the V4.1 file also holds a table of the Calibration object's types, control values and keys (CMD 23). We never send them, so it is only a reference if that decision changes.
- **Baud change:** after a write that changes the baud, they switch their UART 300 ms later. On our TM210 a baud change in RAM never takes effect (see "Baud" above), so the Pico no longer switches after a RAM write.

**A third reader of the same packet format:** the AiO v26 Teensy firmware (https://github.com/AgOpenGPS-Official/Firmware_Teensy_AiO_26, `lib/TM171AiOParser/` and `lib/aio_navigation/IMUProcessor.cpp`, commit `2551bff`, read 2026-11-10, GPL v3). It only reads a TM171 and never writes its settings. Facts from its code; no code is taken.
- **Framing and CRC agree with ours:** `AA 55`, a length byte, 4 header bytes, the payload, then a Modbus CRC-16 (little-endian) over everything between the `AA 55` and the CRC. The object ID is the low 7 bits of the first header byte.
- **It reads only the RPY object** (CMD 35, 20-byte payload: timestamp, roll, pitch, yaw) at 115200 baud, the rate it expects the TM171 to be set to already. We use the raw object (CMD 41) at 921600 instead.
- **Roll is negated by default** before it goes to AgOpenGPS. That fits a sensor frame with y to the left (FLU) against AgOpenGPS's roll, which is positive with the right side down. It is the same flip our FRD conversion makes ([imu-tm210.md](imu-tm210.md)).
- **Its timeouts:** 500 ms to see a valid packet at boot, and the data is marked invalid after 100 ms without one.
- **A guard worth copying as an idea:** before any write they check that bit 13 = 1, bits 14–16 = 2 and the baud word matches the rate they are talking at. If not, they report "layout not recognised" and write nothing. The same check in our provision step would stop a write to a TM210 firmware with a different layout.
- **Baud scan:** they scan 460800, 230400, 921600, 57600, 38400 and 9600 after 115200.

## Target configuration
Read-modify-write. Every field not listed is copied unchanged from the read-back.

"UART1" in this plan is the TM210's own name for its UART (`uart1_Baudrate`, "UART1 output"). On the Pico that link is **UART0**; the Pico's UART1 is the UM980.

| Field | Target | Why |
|---|---|---|
| Status, Quaternion, Euler, RPY, Gravity output (bits 5, 8, 10, 11, 12) | 0 | Raw only. The Pico requests Status at 1 Hz |
| CMD 42 / Combo / unknown (bits 26, 27, 28) | 0 | No extra objects; bit 28 slows output |
| Raw output (bit 6) | 1 | INSLIB input |
| UART1 output (bit 19) | 1 | Link to the Pico |
| USB output (bit 20) | 1 | ImuAssistant on USB for debugging next to the Pico, and the recovery path |
| Temperature calibration (bit 23) | 0 | Vendor recommendation |
| bootMode (bits 24–25) | 0 (Auto) | 3.2 s cold start; INSLIB levels itself |
| UART1 baud (bytes 6–7) | 9216 (921600) | 49 B × 800 Hz ≈ 43 % load |
| Silent Time (bytes 16–17) | 0 | Output starts right away |
| ODR (bytes 18–19) | 800 | Full internal rate |

This target was applied in RAM on the bench and acknowledged. Over USB the TM210 then sent only CMD 41, 1,250 µs apart, with no gaps.

## Boot state machine (core 0, module `imu_tm210_cfg`)
1. **Detect:**
   - Set UART0 to 921600, broadcast the Setting request, and wait up to 300 ms (the vendor's wait) for a CMD 21 reply with a valid CRC.
   - If none comes, scan 115200, 1M, 460800, 230400, 57600, 38400, 9600, 2400 and 1200 the same way. At 2400 and 1200 wait 1 s, because the 29-byte reply alone takes up to about 0.25 s.
   - If nothing answers, report "no IMU" and retry every 2 s. Cold start takes 3.2 s, and longer if a previous user set Static boot.
   - From the reply, keep the node ID (`fromId`) and the raw 20-byte settings.
2. **Check (fast path):**
   - Only when the TM210 was found at 921600.
   - Compare the fields in the target table. Everything else is ignored, including bits 17/18, the sensor enables, the fusion fields, bytes 4–5, 8–9 and 14–15.
   - If they match and CMD 41 arrives at 800 ± 20 Hz over 1 s, configuration is done. Nothing is written to the TM210's flash, and boot takes about 1.3 s longer.
   - **Found at another baud, but the settings already hold the target with 921600:** the Pico saved it earlier and the TM210 has not been power-cycled since. Nothing is written; go to step 3.6.
3. **Provision.** Runs on any other mismatch, or from the "Reconfigure IMU" button on the web page ([web-config-server.md](web-config-server.md)):
   1. **Apply to RAM:** first a layout guard: write only if the read-back has switch bit 13 = 1, bits 14–16 = 0b010 and a baud word that matches the rate in use (the values read on our TM210 and on the AiO firmware's TM171). Otherwise report "layout not recognised" with the raw bytes and stay in the error state. Then build the target from the read-back with bit 17 = 1 and bit 18 = 0, **keeping the baud in use** (a new baud doesn't take effect from RAM). Send it to the node ID at the current baud and wait up to 1.5 s for the Ack (`cmdAck = 21`). Retry up to 5×.
   2. **Read back** at the same baud: the target fields must match, with the baud word still at the rate in use.
   3. **Check the output** (only when already at 921600): only CMD 41 packets, at 800 ± 20 Hz over 2 s, with zero CRC errors. At a lower baud 800 Hz doesn't fit the line, so this is skipped; the output is checked after the power cycle (step 2) instead, before the TM210 is used.
   4. **Commit to flash:**
      - Send the same object with bit 18 = 1 and the baud word set to 921600, wait for the Ack, and read it back once more (still at the old baud; the baud word must now read 921600).
      - At 921600 this only happens after step 3 has passed, so a bad setting never reaches flash. If anything fails before the commit, a power cycle returns the TM210 to its old flash settings. At most 2 flash writes per boot.
   5. Already at 921600: log that boot mode Auto applies from the next power cycle, and go to Run.
   6. **Baud changed: wait for a power cycle.** The state is `power-cycle` and the error reads "921600 baud saved to flash: power-cycle the IMU". Detection runs again every 2 s. Until the TM210 is power-cycled it is found at the old baud with 921600 in its settings, and nothing more is written. After the power cycle it is found at 921600 and step 2 checks the output.
4. **Run:** UART0 RX goes to the CMD 41 parser and the 1 Hz Status request starts ([imu-tm210.md](imu-tm210.md), "Firmware: reading the TM210"). Samples flow to the core 1 queue. Before "Run", INSLIB gets no IMU data.

**How it's built:**
- Same pattern as `gnss_um98x_cfg`: send, wait for the reply, retry, and record each step's result for the diagnostics page. The page shows the state, last error and node ID.
- Each sensor has its own state machine and UART, and they run concurrently on core 0 ([rtos-decision.md](rtos-decision.md)).
- **Packet code:** the encoder/decoder is a small C module written for the firmware, with packed structs and byte-wise CRC access, so it doesn't rely on alignment.
  - It follows the TM210 layout above, not the library's: bytes 16–17 and 18–19 are two uint16 fields, and bits 20 and 26–28 have the meanings found on the bench.
  - The vendor library itself is not compiled in.

## Verification
**Done on the bench (2026-11-06, over USB, RAM only):**
- Settings request and reply.
- Writes to the node ID, acknowledged.
- Output rate at bytes 18–19 (50/200/800 Hz).
- Bit 20 = USB output.
- Bits 26/27 = CMD 42/43 output, and bit 28 slows output.
- The full target applied, giving raw only at 800 Hz with no gaps.
- Status on request.
- Restore to the original settings.

**Done on miniPi and the PC (2026-11-12/13, over USB, RAM only):** details in [imu-tm210.md](imu-tm210.md), "Bench results (2026-11-12/13, TM210 over USB on miniPi and on the Windows PC)".
- **With USB output off in flash (bit 20 = 0), the TM210 still answers Setting and Status requests over USB** and acknowledges a RAM write there, but it streams nothing. So USB is a recovery path even when bit 20 is off.
- The target applied in RAM again on both hosts: 10 min of raw-only output at 802–803 packets per host second, with 0 CRC errors. Whole packets went missing over USB (684 on miniPi, 77 on the PC), most likely on the USB side. That was over USB, so item 1 below is still open, and it also shows whether the TM210 skips samples on its UART.

**Done on dietpi (2026-11-13, Pico UART0, TM210 powered from VBUS):**
- The Ack for a RAM write that changes the baud arrives at the old baud, but the new baud never becomes active (see "Baud" above). This was open item 1; the old provisioning got stuck on it, because its layout guard then saw a baud word that didn't match the rate in use.
- 921600 saved to flash over USB (bit 18 = 1, acknowledged, read back with bit 18 set), then power-cycled: the Pico found it at 921600, the fast path passed with nothing written, and RUN gave 802–804 Hz CMD 41 with 0 CRC errors and 0 gaps. INSLIB ran in ATTITUDE_ONLY with a mean update of 96 µs (max 158 µs).
- With USB connected, the Pico's UART requests went unanswered after a power cycle (see "USB" above).

**Still to do:**
1. **UART link:** 800 ± 20 Hz CMD 41 at 921600 with zero CRC errors over 10 minutes.
2. **Factory-default TM210 (115200):** the Pico applies the target in RAM, saves 921600 to flash and asks for a power cycle; after it the fast path passes. The flash write and the `power-cycle` state are not yet tested by our code (the bench flash write above was done over USB by a script).
3. **Power-cycle:** the fast path passes and there is no flash write (passed once, 2026-11-13). A flash commit at 921600 (bit 18 = 1) has not yet been exercised by our code.
4. **Mismatch:** turn RPY on in ImuAssistant and save. On the next boot it is detected and corrected.
5. **Interrupted provisioning:** remove power between steps 3.1 and 3.4. The next boot finds the old flash settings and provisions again.
6. **USB debugging:** ImuAssistant on USB sees data while the Pico reads the TM210's UART. With USB connected the UART sometimes ignored requests (2026-11-13), so find out when that happens.
7. **Silent Time:** confirm bytes 16–17 are Silent Time in seconds. Set it to 2 in flash, power-cycle, and time the first packet.
