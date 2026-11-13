# IMU alternative: ADIS16465-2BMLZ instead of TM210

## Context
The current IMU is the SYD Dynamics TM210 ([imu-tm210.md](imu-tm210.md)). This plan compares it with the Analog Devices ADIS16465-2BMLZ (±500 °/s variant) and lists what a switch would change in performance and firmware. It is a comparison, not a decision.

Sources: `docs/datasheets/TransducerM_TM210_Datasheet_EN_V126-R.pdf`, `docs/datasheets/adis16465.pdf` (Rev. C, Table 1). The TM210 noise density is derived in [imu-tm210.md](imu-tm210.md).

## Datasheet comparison
| | TM210 | ADIS16465-2 | Ratio |
|---|---|---|---|
| Gyro range | ±1000 °/s | ±500 °/s | enough for a tractor |
| Gyro in-run bias stability | <5.5 °/h | 2.5 °/h (2 °/h is only the -1 model) | ~2× |
| Gyro noise | 0.20–0.35 °/s p-p @100 Hz → ~0.007 °/s/√Hz, ARW ~0.4 °/√h | 0.002 (x) / 0.003 (y, z) °/s/√Hz, ARW 0.15 °/√h | ~2.5–3× |
| Gyro bias over temperature | ±0.004 °/s/°C, calibrated at 22.5 °C only | ±0.2 °/s (1σ) residual over −40…+85 °C | about equal for a 20–40 °C outdoor swing |
| Gyro bias repeatability (long-term) | not specified | 0.4 °/s (500 h HTOL at 105 °C) | |
| Gyro g-sensitivity / vibration rectification | not specified | 0.009 °/s/g / 0.0005 °/s/g² | |
| Gyro scale error over temp / misalignment | not specified | ±0.3 % / ±0.05° | |
| Accel range | ±10 g | ±8 g | |
| Accel noise density | 190 µg/√Hz | 23 µg/√Hz (VRW 0.012 m/s/√h) | ~8× |
| Accel in-run bias stability | 0.05 mg | 3.6 µg | ~14× |
| Accel bias over temperature | not specified | ±1 mg | |
| Accel scale | "in units" (direction accurate, magnitude not) | ±0.1 % over temperature, ±0.05° misalignment | |
| Magnetometer | yes (±1.3 G, 75 Hz) | **none** | |
| Bandwidth | not specified | 550 Hz gyro, 600 Hz accel | |
| Output | UART up to 1 Mbps, 800 Hz, own µs timestamp | SPI (2 MHz, 1 MHz in burst), 2000 SPS with decimation, DR pin, SYNC input, delta-angle/velocity | |
| Power | 5 V, 80 mA | 3.3 V, 44 mA (100 mA start-up transient) | |
| Package | case with PH2.0-5 connector, USB-C | 22.4 × 22.4 × 9 mm, 14-pin 1 mm-pitch connector | |
| Price | low | several hundred USD | |

The clear gains are the **accelerometer** (8–14×), a full factory calibration (scale, misalignment, g-sensitivity), and better timing. The gyro is only 2–3× better. The temperature behaviour is about the same in practice.

## Effect on the system
- **RTK available (normal field work): little visible difference.** Roll accuracy matters most for AgOpenGPS terrain compensation (1° at 3 m antenna height ≈ 5 cm cross-track). Both IMUs reach well under 0.1° once INSLIB has converged the biases. The ADIS converges faster and has less vibration-induced roll noise.
- **Slow speed and stops:** heading comes from gyro integration between COG updates. ZUPT/ZARU re-estimates the gyro bias at every stop, so what remains is the bias stability and ARW. That gives roughly 2× less heading drift with the ADIS.
- **Headland turns:** the ADIS's scale (±0.3 %) and alignment (±0.05°) errors are guaranteed. The TM210's are only as good as the 6-position calibration.
- **GNSS outages (free inertial):** short-term drift is dominated by accel bias, accel noise and tilt error. Expect roughly **3–5× less** position drift with the ADIS. INSLIB's automotive constraints reduce both.
- **Vibration (diesel tractor):** the ADIS specifies and compensates g-sensitivity and vibration rectification. The TM210 specifies neither. This is probably the most noticeable real-world gain. A measurement on the related TM171 supports the concern but also bounds it: in the AgOpenWeb-HAT evaluation (https://github.com/chriskinal/AgOpenWeb-HAT, `hardware/imu-breakout/HARDWARE_IMU_BREAKOUT.md` §8.4, commit `e4bd9ef`, read 2026-11-10) its accelerometer mean moved under heavy shaking (1.5–1.8 g rms, roll off by 3.2°), while at the levels measured on a tractor at rest (18–43 mg rms) the roll shift stayed under 0.14°. See "Vibration check" in [imu-tm210.md](imu-tm210.md).
- **Lost:** the magnetometer. The `nav_suite` AHRS sub-filter never runs, so ATTITUDE_ONLY mode relies on ARS (roll/pitch, yaw free). Since 2026-11-13 the plan feeds the TM210 magnetometer to the AHRS and the INS ([inslib-nav-suite.md](inslib-nav-suite.md)), so with the ADIS the heading at standstill before the first drive (single antenna) and the AHRS fallback would be lost.

## Firmware impact if switched
- **Bus:** SPI1 (SPI0 is reserved for the W5500, which is on hold: [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)). Use a DR-pin GPIO interrupt that starts a DMA burst read. This frees UART0.
- **Rate:** set `DEC_RATE` for 1000 Hz (or 500 Hz). 1000 Hz adds about 25 % to the strapdown cost, still far inside the budget in [rp2350-feasibility.md](rp2350-feasibility.md). The SPI load at 1 MHz burst is fine: about 20 B per sample → 160 kbit/s.
- **Timestamps:** capture the DR edge on the Pico µs clock. This replaces the TM210 offset/drift mapping. Alternatively, drive the SYNC input from a PPS-locked clock, or put DR on the GNSS receiver's EVENT input.
- **Data:** read delta-angle/delta-velocity and divide by dt for INSLIB's `gyr`/`acc`. This handles coning and sculling inside the sensor. Convert to the INSLIB FRD frame according to the axis marks on the ADIS case.
- **Calibration:** Allan variance (for the `Qll_diag` PSDs and bias random walks) and the mount angles are still needed. The 6-position accelerometer-magnitude fix and magnetometer calibration are not.
- **Start values for INSLIB:** `acc.Qll_diag` ≈ (23 µg/√Hz × 9.81)² ≈ 5.1e-8 (m/s²)²/Hz. `gyr.Qll_diag` ≈ (0.003 °/s/√Hz in rad)² ≈ 2.7e-9 (rad/s)²/Hz.
- **Hardware:** 3.3 V supply that can handle the 100 mA start-up transient. A breakout board or cable for the 1 mm-pitch connector.

## Bottom line
The ADIS16465-2 is a moderate gyro upgrade and a large accelerometer, calibration and vibration upgrade. It pays off when coasting through GNSS outages (tree lines, buildings) and on rough running. In open-field RTK steering it looks almost the same as the TM210. The costs are the price, a new SPI driver, and losing the magnetometer.

## Verification (if hardware is bought)
1. Allan variance on both IMUs, 2 h static at constant temperature (`external/INSLIB/tools/allan_variance.py`). Real figures often differ from datasheets.
2. Mount both on the same vehicle and log a drive with tree-line passes, all on the Pico clock.
3. Replay each IMU with the same GNSS through `external/INSLIB/tools/replay.c`. Cut GNSS for 10 s and 30 s, then compare position, roll and heading error against the RTK truth.
