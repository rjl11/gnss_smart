# IMU: TM210 auto-configuration by the MCU

## Context
The user should only have to connect a TM210, new or previously used, and the Pico configures it. No PC and no ImuAssistant. This plan covers:
- the EasyProtocol objects involved
- the target settings
- the boot-time state machine that detects, checks and provisions the IMU

It follows the same pattern as the UM980 ([gnss-um980-config.md](gnss-um980-config.md)). The resulting IMU data and its conversion for INSLIB are in [imu-tm210.md](imu-tm210.md).

Sources:
- `docs/datasheets/TransducerM_TM3xx_UserGuide_EN_V135-R1.pdf`: EasyProtocol objects Request 12, Acknowledge 13, Setting 21, Status 22, Calibration 23, Raw 41. Also the sections "Communication Protocol" and "Change UART Baudrate".
- `docs/datasheets/TransducerM_TM210_Datasheet_EN_V126-R.pdf`: ODR values, baud range, start-up times.
- `docs/TransducerM_AN002_EN_V11.pdf`: run-time forced calibration for AGVs.
- SYD Dynamics Q&A, https://www.syd-dynamics.com/questions-answers/ (read 2026-11-06): ODR vs Inhibit Time per model, baud recovery per model, the effect of the sensor enable checkboxes, calibration.
- SYD Dynamics example code in `external/syd_dynamics/` (vendored, not compiled; overview in its `README.md`):
  - `TransducerM_Example_CPP_QT_V131-R1/EasyProfile/EasyObjectDictionary.h` (EasyProfile V1-3-1-R1, 2025): the current `Ep_Settings`, `Ep_Switches` and `Ep_Combo` definitions.
  - `TransducerM_Example_CPP_QT_V131-R1/HelloMotionModule.cpp`, `Example_ChangeSetting()`: the vendor's read-modify-write sequence for settings.
  - `TransducerM_Example_RaspberryPiPico_V1-1/`, `TransducerM_Example_STM32_CPP/`, `TransducerM_Example_ROS-pkg_V1-9/`, `TransducerM_Example_ROS2-pkg_V1-1/`, `TransducerM_Protocol_C_V1-2-1-Beta/`: embedded, Linux and C builds of the same library.
    - None of them write settings.
    - The STM32 example receives with `HAL_UARTEx_ReceiveToIdle_DMA`, the same DMA-plus-idle pattern planned for UART0.
    - Its EasyProtocol writes the CRC byte by byte, which avoids unaligned access.

## Facts that shape the design
### Packet and addressing
- **Layout:** `AA 55 | len | 4-byte header | payload | CRC16 Modbus`. The CRC covers len + header + payload and is stored little-endian.
- **Header bits, LSB first:** CMD (7), QoS (3), fromId (11), toId (11).
  - The user guide and the QT library call the 3 bits "reserved, must be zero".
  - The newer embedded builds (Pico, STM32, ROS) call them "Quality-of-Service of the source device (only available on certain Models)", and the vendor parser doesn't reject non-zero values.
  - So **our parser accepts any value** there and reports it as a QoS hint. The host sends 0.
- **IDs:** host = 2, broadcast = 0, unspecified = 1 (`EP_ID_*` in `EasyObjectDictionary.h`).

### Reading and writing settings
- **Read:** send a Request (CMD 12, payload `{cmdRequest = 21, reserved[3]}`). Broadcasting it is allowed. The TM210 answers with its Setting object, and the reply's `fromId` is the TM210's node ID.
- **Write** (the vendor's `Example_ChangeSetting()`):
  1. Request the current settings.
  2. Change only the wanted fields and set `request_acknowledge = 1`.
  3. Send to `toId = <fromId of the reply>`. The vendor comment says "EP_ID_BROADCAST_ should not be used here".
  4. Wait up to 1.5 s for an Acknowledge (CMD 13) with `cmdAck = 21`. The vendor retries up to 5×.
- **Read-modify-write is required.** The guide says not to change reserved fields: "changing of these reserved fields may result in unexpected behavior of the sensor module".

### `Ep_Settings` (20-byte payload)
| Bytes | Field | Notes |
|---|---|---|
| 0–3 | `switches` | bit fields, see below |
| 4–5 | `reserved` | must stay 1152 |
| 6–7 | `uart1_Baudrate` | unit 100 bps: 9216 = 921600, 1152 = 115200 |
| 8–9 | `can_Baudrate` | "should always be 0" |
| 10–11 | `gain_Acc` | ×0.01, fusion only |
| 12–13 | `gain_Mag` | ×0.01, fusion only |
| 14–15 | `inhibitTime` | ms; on the TM210 this is where ODR goes (see below) |
| 16–19 | `silentTime` | uint32, seconds |

### `Ep_Switches` (bits LSB first)
| Bit | Field | Notes |
|---|---|---|
| 0–2 | gyro / acc / mag enable | |
| 3 | GyroErrFilter | fusion |
| 4 | MagSelfAdaptFilter | fusion |
| 5 | Status output | continuous |
| 6 | Raw output | continuous |
| 7 | reserved1 | must be 0 |
| 8 | Quaternion output | |
| 9 | reserved2 | must be 0 |
| 10 | Euler output | |
| 11 | RPY output | |
| 12 | Gravity output | |
| 13 | reserved3 | must be 1 |
| 14–16 | reserved4 | must be 0b010 |
| 17 | request_acknowledge | |
| 18 | save_Setting_Permanently | 0 = RAM only, 1 = flash |
| 19 | enable_UART1 | |
| 20 | reserved5 | must be 0 |
| 21 | enable_USB1_CAN1 | on the TM210 this is the USB port |
| 22 | reserved6 | must be 0 |
| 23 | reserved7 | "Apply Temperature Calibration. Set to 0 is recommended." |
| 24–25 | bootMode | 0 = Auto, 1 = Static, 2 = Dynamic |
| 26–31 | unused | must be 0 |

### Other constraints
- **Flash or RAM:**
  - `save_Setting_Permanently = 0` applies the settings in RAM only. The guide calls this "safer than programming the flash memory", and flash needs a stable supply.
  - Boot mode only takes effect after a power cycle, and only if it was saved to flash.
- **ODR is not documented** in the guide, the datasheet or any library version.
  - The guide only says that some models show ODR in place of Inhibit Time. The TM210 offers 800, 400, 200, 100, 50, 25, 10, 5 and 1 Hz.
  - The SYD Q&A confirms that the TM210 is one of the ODR models (it has no Inhibit Time) and that 800 Hz is available on it. It still doesn't give the encoding.
  - How ODR is encoded in bytes 14–15 must be read from a TM210 set to 800 Hz in ImuAssistant (verification step 1). The result becomes a firmware constant.
- **Baud:**
  - The factory default is 115200. Supported rates are 1M, 921600, 460800, 230400, 115200, 57600, 38400, 9600, 2400 and 1200.
  - The datasheet recommends ≥921600 for ODR ≥ 400 Hz. The Q&A asks for ≥230400 or ≥460800 at 400/800 Hz, "depending on the number of different data types selected for output". 921600 with raw output only is well inside both.
  - **Recovery when the baud is unknown:** for the TM210 the vendor's answer is the USB port ("simply use its USB interface to connect and find or reset the baudrate"). The 6-second 115200 SCAN lock in the user guide applies to the TM200/TM35x only. The firmware scans the UART baud rates itself, and USB stays enabled as the manual fallback.
- **Sensor enable bits (gyro/acc/mag):** per the Q&A they choose what goes into the TM210's own fusion (Quaternion, and from it Euler/RPY/Gravity), and they "do not affect Raw Accelerometer, Gyroscope and Magnetometer output". So they are left as they are and not checked.
- **Calibration commands** (CMD 23: CalibB, AN002 forced calibration, heading fix, compass calibration) affect only the TM210's own fused outputs. **We use raw data only, so the firmware never sends them.** INSLIB estimates the biases itself.
- **`Ep_Combo` (CMD 43, new in 2025)** is a packed set of integers: gyro in 1e-5 rad/s, acc in 1e-5 g with the library's 1 g = 9.794 m/s², plus QoS, temperature and rate. It is not used. Raw (CMD 41, float32) keeps full resolution and is what [imu-tm210.md](imu-tm210.md) is built on.
- **Start-up:** Auto boot gives a 3.2 s cold start, and Static boot 10–30 s. The TM210 answers requests even during its silent time.

## Target configuration
Read-modify-write. Every field not listed is copied unchanged from the read-back.

| Field | Target | Why |
|---|---|---|
| gyro / acc / mag enable | unchanged (copied, not checked) | Q&A: no effect on raw output |
| Status, Quaternion, Euler, RPY, Gravity output | 0 | Raw only. The Pico requests Status at 1 Hz |
| Raw output | 1 | INSLIB input |
| enable_UART1 | 1 | Link to the Pico |
| enable_USB1_CAN1 | 1 | ImuAssistant on USB for debugging, alongside the Pico. Also the TM210's recovery path if the UART baud is lost |
| reserved7 (temperature calibration) | 0 | Vendor recommendation |
| bootMode | 0 (Auto) | 3.2 s cold start; INSLIB levels itself |
| uart1_Baudrate | 9216 (921600) | 49 B × 800 Hz ≈ 43 % load |
| inhibitTime (ODR field) | 800 Hz value from the bench capture | ODR 800 Hz |
| silentTime | 0 | Output starts right away |
| reserved, reserved1–6, gains, fusion switches | unchanged | Vendor: do not change |

## Boot state machine (core 0, module `imu_tm210_cfg`)
1. **Detect:**
   - Set UART0 to 921600, broadcast the Setting request, and wait up to 300 ms (the vendor's wait) for a CMD 21 reply with a valid CRC.
   - If none comes, scan 115200, 1M, 460800, 230400, 57600, 38400 and 9600 the same way.
   - If nothing answers, report "no IMU" and retry every 2 s. Cold start takes 3.2 s, and longer if a previous user set Static boot.
   - From the reply, keep the node ID (`fromId`) and the raw 20-byte settings.
2. **Check (fast path):**
   - Only when the TM210 was found at 921600.
   - Compare the target fields: the output switches (Status, Raw, Quaternion, Euler, RPY, Gravity), UART1 enable, USB enable, temperature calibration, boot mode, baud, ODR and silent time. The sensor enables and the fusion fields are not compared. If they match and CMD 41 arrives at 800 ± 20 Hz over 1 s, configuration is done.
   - Nothing is written to the TM210's flash, and boot takes about 1.3 s longer.
3. **Provision.** Runs on any mismatch, or from the "Reconfigure IMU" button on the web page ([web-config-server.md](web-config-server.md)):
   1. **Apply to RAM:**
      - Build the target from the read-back with `save_Setting_Permanently = 0` and `request_acknowledge = 1`.
      - Send it to the node ID at the current baud and wait up to 1.5 s for the Ack (`cmdAck = 21`). Retry up to 5×.
   2. **New baud:** if the baud changed, switch UART0 to 921600 and wait 50 ms. Read the settings back: the target fields must match.
   3. **Check the output:** only CMD 41 packets, at 800 ± 20 Hz over 2 s, with zero CRC errors.
   4. **Commit to flash:**
      - Send the same object with `save_Setting_Permanently = 1`, wait for the Ack, and read it back once more.
      - This only happens after step 3 has passed, so a bad setting never reaches flash. If anything fails before the commit, a power cycle returns the TM210 to its old flash settings.
   5. Log that boot mode Auto applies from the next power cycle.
4. **Run:** UART0 RX goes to the CMD 41 parser and the 1 Hz Status request starts ([imu-tm210.md](imu-tm210.md), "Firmware: reading the TM210"). Samples flow to the core 1 queue. Before "Run", INSLIB gets no IMU data.

**How it's built:**
- Same pattern as `gnss_um980_cfg`: send, wait for the reply, retry, and record each step's result for the diagnostics page. The page shows the state, last error and node ID.
- Each sensor has its own state machine and UART, and they run concurrently on core 0 ([rtos-decision.md](rtos-decision.md)).
- **Packet code:** the encoder/decoder is a small C module written for the firmware. It follows `EasyObjectDictionary.h` V1-3-1 with packed structs and byte-wise CRC access, so it doesn't rely on alignment. The vendor library itself is not compiled in.

## Verification (bench, when implemented)
1. **ODR capture**, before the firmware constants are written:
   - Set 800 Hz in ImuAssistant, request the Setting object from a serial terminal, and record bytes 14–15. Repeat at 200 Hz to learn the encoding.
   - Check the switch bit layout against the same read-back.
2. **Factory-default TM210:** provisioned with no user action. It ends at 921600 with CMD 41 only, at 800 ± 20 Hz.
3. **Power-cycle:** the fast path passes and there is no flash write.
4. **Mismatch:** turn RPY on in ImuAssistant and save. On the next boot it is detected and corrected.
5. **Interrupted provisioning:** remove power between steps 3.1 and 3.4. The next boot finds the old flash settings and provisions again.
6. **USB debugging:** ImuAssistant on USB sees data while the Pico reads UART1.
7. **Open questions:**
   - The ODR encoding.
   - Whether a baud change takes effect right after the Ack, or only after a restart. If only after a restart, commit to flash before switching baud.
   - At which baud the Ack arrives (old or new).
