# RP2350 feasibility for INSLIB nav_suite (GNSS 20 Hz, IMU 800 Hz, baro 100 Hz, 10 Hz out)

## Context
The MCU isn't chosen yet. The question is whether an RP2350 (2x Cortex-M33 at 150 MHz, single-precision FPU, double-precision coprocessor, 520 KB SRAM) can run INSLIB's `nav_suite` at these sensor rates.

## Answer: yes, with a large margin (estimate, not yet measured)
What INSLIB says about itself:
- Its reference target is a Cortex-M4F at 180 MHz (STM32F429). The library is designed for this class of MCU: no heap, bounded run time, 32-bit float everywhere except GNSS coordinates (`readme.md`, `requirements/req_sys.md`).
- The strapdown runs at the full IMU rate, but the covariance prediction runs at `kalman_update_dt_sec`, which defaults to 20 Hz (`src/ins.h`). So 800 Hz IMU does **not** mean 800 Kalman predictions per second.
- KFCore benchmark on the M4F: prediction takes about 400–600 µs and update about 100–135 µs (`KFCore/README.md`).
- The worst-case stack for `nav_suite_update()` is about 13 KB (readme, x86_64 figure). That's trivial against 520 KB of SRAM.

Rough CPU budget, scaling the M4F figures to an M33 at 150 MHz:

| Task | Rate | Est. cost | CPU/s |
|---|---|---|---|
| Strapdown mechanisation | 800 Hz | ~15–25 µs | ~2 % |
| Covariance prediction | 20 Hz | ~0.7–1 ms | ~2 % |
| GNSS update (pos+vel) | 20 Hz | ~0.3–0.6 ms | ~1 % |
| Baro altitude filter (3 states) | 100 Hz | <50 µs | <0.5 % |
| AHRS inside nav_suite | 800 Hz / 20 Hz | similar to the INS | ~2–3 % |
| Output 10 Hz | 10 Hz | negligible | — |

That totals roughly **5–10 % of one core**. The second core is free for parsing and I/O. The RP2350's double-precision coprocessor makes the few double operations (lat/lon) cheaper than on the M4F reference.

## Real risks (I/O and timing, not compute)
1. **TM210 at 800 Hz:** this is about 30–50 kB/s depending on packet format. It needs ≥460800 baud UART (or SPI), received via DMA. Check the max output rate and interface in `docs/TransducerM_TM210_Datasheet_EN_V126-R.pdf`.
2. **UM980 at 20 Hz:** use Unicore binary logs (e.g. PVTSLN/BESTNAV) at ≥460800 baud rather than heavy NMEA. Also use PPS so GNSS timestamps line up with IMU time.
3. **Timestamping:** INSLIB tolerates measurement delay through a history buffer (`ins.h`). Every sample still needs an accurate capture timestamp, so take it in the ISR or DMA-complete handler.
4. **Execution from flash:** the RP2350 executes code from external QSPI flash through a cache (XIP). To get deterministic timing, put the filter's hot path in SRAM (`__not_in_flash_func` / `copy_to_ram` build).

## Verification (when implementing)
1. Cross-compile `src/*.c` and `KFCore/c/*.c` with the Pico SDK (arm-none-eabi-gcc, `-O2 -mcpu=cortex-m33 -mfloat-abi=hard`).
2. Write a bench firmware that replays a recorded INSLIB dataset (`datasets/`) at 800/100/20 Hz through `nav_suite_update()`. It should measure per-call cycles with the M33 DWT cycle counter (worst case and mean) and print the results over USB.
3. Pass criterion: worst case per 1.25 ms IMU tick (with a GNSS update and covariance prediction in the same tick) stays well under 1.25 ms, and average load is below 30 % of one core.
4. Check that the output matches a host build of `replay` on the same dataset.
