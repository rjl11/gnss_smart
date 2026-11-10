# INSLIB integration: use nav_suite

## Context
The filter is INSLIB (`external/INSLIB`). It can be used as the bare 15/18-state `ins` filter or through the `nav_suite` wrapper. This plan commits to **`nav_suite`** and describes how the firmware feeds it and reads it. IMU details are in [imu-tm210.md](imu-tm210.md). CPU and timing are in [rp2350-feasibility.md](rp2350-feasibility.md).

## Why nav_suite and not bare ins
`nav_suite_update()` (`external/INSLIB/src/nav_suite.h`) drives five filters from one measurement epoch:
- **ins**: full navigation filter (position, velocity, attitude, IMU biases).
- **ars**: roll/pitch filter, yaw free (directional gyro).
- **ahrs**: roll/pitch/yaw with the magnetometer (only runs once a mag sample has been seen).
- **baro_alt**: baro/accelerometer vertical channel (BMP581).
- **local_gnss**: offset between the local height and the WGS84 ellipsoid, so absolute height survives a GNSS outage.

What that buys over bare `ins`:
- **Graceful degradation**: `nav_suite_get_mode()` returns FULL, COASTING (dead reckoning), ATTITUDE_ONLY (INS lost, AHRS still gives attitude) or NONE. Re-acquisition after an outage is automatic.
- **Best-source getters**: `nav_suite_get_rpy()`, `nav_suite_get_height()`, `nav_suite_get_height_ellipsoid()`.
- **Heading carry-over**: after an INS re-init, yaw is restored from the last INS heading plus the ARS yaw change (REQ-SUITE-022).
- **One place for shared settings**: outlier rejection and stillness (auto-ZUPT) thresholds are copied from `ins_options_t` into every sub-filter by `nav_suite_init()`.
- Same constraints as bare `ins`: no heap, all state in `nav_suite_t`, worst-case stack about 13 KB.

## Lifecycle (core 1)
1. Wait for the receiver model from core 0 (UM980 or UM982), because it sets `automotive_mode`. The wait ends after 10 s at the latest (start value, not yet measured), with the UM980 setting; a UM982 identified later triggers a re-init ([gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md)). Then zero a static `nav_suite_t`, fill `ins_init_t` and `ins_options_t`, call `nav_suite_init()`.
2. Optionally tune `suite.ars_cfg` / `suite.ahrs_cfg` / `suite.baro_cfg` before the first update.
3. Loop: pop one IMU sample (plus any GNSS/baro that arrived) from the core 0 queue, build an `ins_measurements_t`, call `nav_suite_update()`. One call per TM210 sample, so 800 calls per second. The strapdown runs every call; the covariance prediction runs at `kalman_update_dt_sec` (20 Hz).
4. After the first usable GNSS fix, and again after moving far (e.g. 50 km), call `nav_suite_set_magnetic_model_position(lat, lon, year)`.
5. Every 100 ms, copy the outputs into a struct that core 0 reads (BLE, web page, and `$PAOGI` once Ethernet is back from on hold).

## Options for a tractor (AgOpenWeb) setup
| `ins_options_t` field | Value | Why |
|---|---|---|
| `auto_init` | true | Bootstrap from the first good fix; roll/pitch from leveling |
| `automotive_mode` | true on a UM980, false on a UM982 | Yaw from GNSS course over ground. The UM980 is single-antenna, so this is the main heading source when moving. The UM982 variant feeds its dual-antenna heading into `m.yaw` instead ([gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md)) |
| `automotive_min_speed_mps` | 0 (default 2 m/s) to start; try ~1 m/s for slow field work | Below it the course is noise |
| `kalman_update_dt_sec` | 0 (= 20 Hz) | Covariance prediction rate |
| `auto_zupt_disable` | false | Bias re-estimation whenever the tractor stands still |
| `gnss_pos_cov_scale`, `gnss_pos_stddev_floor_*` | tune from RTK logs | Receiver σ is often optimistic |
| `automotive_lateral_constraint` | false until the mount is calibrated | Needs a correct mounting matrix |
| `imu_acc_misalignment` / `imu_gyr_misalignment`, `imu_*_fixed_bias` | from IMU calibration + mount angles | See [imu-tm210.md](imu-tm210.md) |
| `mag_field_check_disable` / magnetometer | mag not fed to the INS at first | A steel tractor disturbs it; enable after on-vehicle calibration |

`ins_init_t`: leave position at zero (auto-init replaces it), set the bias random walks and initial σ from the IMU plan; leave other fields at 0 for INSLIB defaults.

## Building one epoch
Zero `ins_measurements_t` every epoch. Set `is_valid` only on an epoch that carries a **new** sample (a latched flag re-fuses the same reading and makes the filter overconfident).

| Field | Source | Notes |
|---|---|---|
| `timestamp` | TM210 sample time on the Pico µs clock | 64-bit, monotonic |
| `strapdown_dt_sec` | difference of TM210 timestamps | ~1.25 ms |
| `acc`, `gyr` | TM210 raw object, converted to FRD SI | `Qll_diag` = PSD, see IMU plan |
| `mag` | TM210 raw mag, only when the value changed | Off for the INS at first |
| `gnss_pos` | UM980 BESTNAVB at 20 Hz ([gnss-um980-config.md](gnss-um980-config.md)): lat/lon in rad, ellipsoid height = hgt (MSL) + undulation, σ → diagonal `Qll_ned` | Only when solution status is usable |
| `gnss_vel` | UM980 horizontal speed + track + vertical speed → NED, σ → diagonal `Qll_ned` | |
| `gnss_leverarm_b` | antenna position relative to the IMU, FRD [m] | Measure on the vehicle |
| `gnss_delay_ms` / `gnss_delay_us` | IMU epoch time − GNSS time of validity | GNSS time from PPS; INSLIB's history buffer handles the delay |
| `yaw`, `yaw_delay_ms` / `yaw_delay_us` | UM982 only: UNIHEADINGB heading minus `baseline_yaw_b`, at 20 Hz ([gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md)) | Left invalid on a UM980. Same delay as the GNSS epoch |
| `baro` | BMP581 pressure [Pa], 100 Hz | New samples only |

GNSS and baro samples are attached to the next IMU epoch after they arrive on core 1; the delay fields carry how old they are.

## Outputs (10 Hz)
`$PAOGI` is the AgOpenWeb sentence sent over Ethernet, which is on hold ([ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md)). For now, BLE NMEA is the only output.

| Output | INSLIB call | Used by |
|---|---|---|
| Lat/lon/height | `ins` local position → `nav_suite_local_to_wgs84()`, height from `nav_suite_get_height_ellipsoid()`. GGA and `$PAOGI` carry height above mean sea level, so subtract the undulation from BESTNAVB (and put it in GGA's geoid separation field) | `$PAOGI`, BLE NMEA |
| Heading, roll, pitch | `nav_suite_get_rpy()` | `$PAOGI` (AgOpenWeb uses heading and roll) |
| Speed | INS NED velocity → horizontal speed | `$PAOGI`, RMC |
| Fix quality | `nav_suite_get_mode()` combined with UM980 RTK status | GGA quality field: FULL keeps the RTK status, COASTING reports dead reckoning. In ATTITUDE_ONLY/NONE, GGA carries the UM980's own position and quality while the receiver has a fix, because SW Maps forwards GGA to the caster ([rtcm-over-bluetooth.md](rtcm-over-bluetooth.md)); it reports no fix only when the receiver has none too |

Antenna vs IMU vs vehicle reference point: output the position of the point AgOpenWeb expects (lever arm applied), not the IMU position.

## Diagnostics (web page)
Mode, `nav_suite_get_zaru_active()`, `nav_suite_get_vertical_zupt_active()`, `suite.n_imu_loss`, INS vs ARS/AHRS yaw difference, estimated gyro/acc biases, IMU temperature from the TM210 status object.

## Verification
1. Record a drive: TM210 raw, UM980 logs and BMP581 pressure, all timestamped on the Pico clock (SD card or USB). Convert to INSLIB's CSV layout (`external/INSLIB/doc/example_imu.csv`, `example_gnss.csv`).
2. Run the host `tools/replay.c` build on it and check the trajectory and heading (`tools/ins_kml.py`, `tools/ins_plots.py`).
3. Feed the same log through the on-target build and compare with the host result.
4. Simulate a GNSS outage (cut GNSS for 10–30 s in the log) and check that the mode drops to COASTING and recovers to FULL.
5. Timing: the cycle-count bench in [rp2350-feasibility.md](rp2350-feasibility.md).
