# IMU: TM210 configuration for INSLIB

## Context
The IMU is a SYD Dynamics TransducerM **TM210** (9-axis AHRS). INSLIB does its own fusion ([inslib-nav-suite.md](inslib-nav-suite.md)), so the TM210 is used only as a source of calibrated raw gyro, accelerometer and magnetometer samples. This plan covers how to configure it, how to read it on the RP2350, and how to convert its data into what INSLIB expects.

Sources: `docs/datasheets/TransducerM_TM210_Datasheet_EN_V126-R.pdf`, `docs/TransducerM_TM3xx_UserGuide_EN_V135-R1.pdf` (EasyProtocol section; the protocol is shared across TransducerM models). Vendor example code and the TM210 3D model (`TransducerM_TM210_3D-Shape_V1-1-2.stp`, for mounting) are in `external/syd_dynamics/`. Vendor Q&A: https://www.syd-dynamics.com/questions-answers/.

## Key facts
- Internal update rate 800 Hz. Output data rate (ODR) selectable: 800, 400, 200, 100, 50, 25, 10, 5, 1 Hz.
- UART: 3.3 V TTL (5 V tolerant), 2400 bps to 921600 / 1 Mbps. The vendor recommends ≥921600 for ODR ≥400 Hz. USB-C virtual COM port works at the same time.
- Power: 5 V, 80 mA. Connector PH2.0-5: RXD, TXD, 5V, GND, GND.
- **Sensor frame** (printed on the case): x forward, y left, z up (FLU). INSLIB works in **FRD** (x forward, y right, z down).
- Gyro: ±1000 °/s, noise 0.20–0.35 °/s peak-to-peak at 100 Hz, bias stability <5.5 °/h, temperature drift ±0.004 °/s/°C.
- Accelerometer: ±10 g, noise density 190 µg/√Hz, bias instability 0.05 mg. Calibrated "in units": direction is accurate, absolute magnitude is not.
- Magnetometer: ±1.3 G, internal sampling 75 Hz, unit = "one Earth field as measured at factory calibration in Denmark".
- Calibrated at 22.5 °C only (TM210-T1 variant: 5–65 °C). Outdoors on a tractor, expect temperature-driven gyro bias that INSLIB must track.

## EasyProtocol packet
`AA 55 | length (1 B) | header (4 B) | payload | CRC16 (2 B, Modbus, over length + header + payload)`

Header bit fields: CMD (7 bits), 3 bits, FROM_ID (11), TO_ID (11). The user guide calls the 3 bits "reserved, must be 0", and the newer embedded EasyProfile builds call them QoS. On the TM210 they read 0 on every packet, so accept any value and ignore it. QoS comes from the Status object ([imu-tm210-config.md](imu-tm210-config.md)).

**Raw Sensor Data object, CMD 41** (40-byte payload, 49 bytes per packet):
| Bytes | Field | Unit |
|---|---|---|
| 0–3 | timestamp, uint32 | µs since TM210 start, wraps every ~71.6 min |
| 4–15 | gyro x, y, z (float32) | rad/s |
| 16–27 | acc x, y, z (float32) | g (vendor: 1 g = 9.8158 m/s²) |
| 28–39 | mag x, y, z (float32) | Earth-field units |

**Status object, CMD 22**: timestamp, temperature [°C], internal update rate [Hz], QoS (0–5). Request with the broadcast command `aa55080c08000016000000e0ed`.

## TM210 configuration (set by the MCU at boot)
The Pico checks these settings at every boot and provisions the TM210 if they don't match: read-modify-write of the Setting object, applied to RAM first and saved to flash only after the output has been verified. No PC is needed. Details are in [imu-tm210-config.md](imu-tm210-config.md).

| Setting | Value | Why |
|---|---|---|
| Output objects | **Raw data only**. Status, RPY, Quaternion, Euler, Gravity, Combo (CMD 43) and CMD 42 off | INSLIB fuses itself; less traffic and parsing |
| ODR | 800 Hz (Setting bytes 18–19, in Hz; verified on the bench) | Full internal rate; INSLIB strapdown at 800 Hz |
| TM210 UART baud (the TM210 calls its port UART1; it goes to the Pico's UART0) | 921600 | 49 B × 800 Hz = 39.2 kB/s ≈ 392 kbit/s with start/stop bits, ~43 % of 921600 |
| USB output (switch bit 20) | on | ImuAssistant can debug over USB while the Pico reads UART; also the recovery path. The TM210 has no CAN bus, so the user guide's CAN settings don't apply |
| Temperature calibration (`reserved7`) | 0 | Vendor recommendation |
| Boot mode | Auto boot (3.2 s cold start) | No need for the 10–30 s static boot; INSLIB levels itself |
| Calibration commands (CalibB, forced calibration per AN002, heading fix) | never sent | They only affect the TM210's fused outputs. With raw-only output they are irrelevant, and INSLIB estimates the biases |
| Sensor fusion profile, gains, gyro/acc/mag enable checkboxes | unchanged | Do not affect raw data output (the vendor Q&A says so for the enable checkboxes) |
| Status object | not auto-output; Pico requests it at 1 Hz | Temperature and QoS for logs and the web page |

Recovery: if the baud rate is unknown, the firmware scans the supported rates itself. By hand, the TM210 is recovered over its USB port (vendor Q&A); the 6-second 115200 SCAN lock in the user guide is for the TM200/TM35x, not the TM210. ImuAssistant is still used for bench work (Allan variance, debugging).

Check after configuring: the TM210 sends only CMD 41 packets at 800 ± 20 Hz on its UART (the Pico's UART0).

## Firmware: reading the TM210 (core 0)
- **UART0** at 921600, RX into a DMA ring buffer (≥4 KB, ~100 ms of data). TX carries the Setting read and write at boot ([imu-tm210-config.md](imu-tm210-config.md)) and then the 1 Hz status request.
- **Parser**: sync on `AA 55`, check length and CRC, accept and ignore the 3 header bits after CMD, accept CMD 41 (plus 21, 13 and 22 when requested or during configuration). Count CRC errors and lost packets for diagnostics.
- **Timestamps**:
  - Unwrap the uint32 µs timestamp to 64-bit.
  - Use TM210 timestamp differences for `strapdown_dt_sec` (the sensor's own sample spacing, no UART jitter).
  - Map TM210 time to the Pico µs clock with an offset + drift estimate: offset = minimum of (arrival time − TM210 time) over a sliding window, minus the fixed packet transmit time (~0.53 ms at 921600). GNSS (PPS) and baro are on the Pico clock, so everything shares one timebase.
  - Flag gaps (dt > 2 × 1.25 ms) and backwards steps; INSLIB's IMU-loss handling takes over on long gaps.
- Push one sample per packet onto the core 0 → core 1 queue.

## Converting to INSLIB units and frame
INSLIB wants FRD, accelerometer as **specific force** in m/s² (level and at rest: `acc ≈ (0, 0, −9.81)`, see `external/INSLIB/doc/example_imu.csv`), gyro in rad/s, magnetometer in µT.

| Sensor | TM210 (FLU) → INSLIB (FRD) |
|---|---|
| Gyro | `(gx, −gy, −gz)` rad/s |
| Accelerometer | `(ax, −ay, −az) × 9.8158` **if** the TM210 outputs specific force. The datasheet footnote shows `(0, 0, −1)` with z pointing to the sky, which is the gravity-vector sign; then it is `(−ax, ay, az) × 9.8158`. **Decide on the bench** (see checks below). |
| Magnetometer | `(mx, −my, −mz) × ~50 µT` (approx. total field in Denmark), then refine with mag calibration |

Do the sign/unit conversion in the parser so all later data is plain FRD SI. Put the **physical mounting** (TM210 not aligned with the vehicle) into config as `mount_rpy_deg` and compose it into `imu_acc_misalignment` / `imu_gyr_misalignment` with `imu_mount_compose()` from `external/INSLIB/tools/imu_mount.h`.

Magnetometer: the TM210 samples it at 75 Hz but repeats it in every 800 Hz packet. Set `mag.is_valid` only when the value changes. Keep it off for the INS until it is calibrated on the vehicle ([inslib-nav-suite.md](inslib-nav-suite.md)).

## INSLIB noise parameters
Start values from the datasheet, to be replaced by Allan variance results:
| Parameter | Start value | Derivation |
|---|---|---|
| `acc.Qll_diag` (PSD) | 3.5e-6 (m/s²)²/Hz | (190 µg/√Hz × 9.81)² |
| `gyr.Qll_diag` (PSD) | 1.5e-8 (rad/s)²/Hz | 0.2–0.35 °/s p-p at 100 Hz → σ ≈ 0.05 °/s over 50 Hz → ~1.2e-4 rad/s/√Hz |
| `gyr_bias_pred_stddev_rps_sqrts` | 2e-6 to 1e-5 rad/s/√s | `config_basic.yaml` value, raised for outdoor temperature swing |
| `acc_bias_pred_stddev_mps2_sqrts` | 3e-4 m/s²/√s | `config_basic.yaml` value |

## Calibration steps
1. **Allan variance**: 2 h static log at 800 Hz, constant temperature → `external/INSLIB/tools/allan_variance.py` → final PSDs and bias random walks.
2. **6-position static test** (each axis up and down, ~30 s each) → `external/INSLIB/tools/inslib_imu_calib.py` → accelerometer scale and fixed bias, gyro fixed bias, into `imu_acc_misalignment` / `imu_*_fixed_bias`. This also fixes the "units, not absolute" accelerometer magnitude.
3. **Mount angles** on the vehicle: `external/INSLIB/tools/inslib_frame_align.py` or measure, set `mount_rpy_deg`.
4. **Magnetometer** (later, on the vehicle): `external/INSLIB/tools/inslib_mag_calib.py` → `mag_misalignment`, `mag_fixed_bias`.

## Bench checks (after conversion, FRD)
Expected specific force: `f = (g·sinθ, −g·sinφ·cosθ, −g·cosφ·cosθ)` for pitch θ, roll φ.
- Level → acc ≈ (0, 0, −9.81). If z comes out +9.81, the accelerometer sign is the gravity-vector convention: switch to the second row of the table.
- Nose up → acc_x > 0. Right side down → acc_y < 0.
- Turn right (clockwise seen from above) → gyr_z > 0. Roll right → gyr_x > 0. Nose up → gyr_y > 0.
- Rate: 800 ± 20 packets/s, CRC error rate ≈ 0 over 10 minutes with the real cable length.
- Timestamp wrap: run > 72 min and check the unwrapped time stays monotonic.
- Run INSLIB's static auto-init on the bench: roll/pitch converge to the table angle, mode reaches ATTITUDE_ONLY without GNSS.
