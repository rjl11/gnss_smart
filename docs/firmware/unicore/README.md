# Unicore UM980 / UM982 receiver firmware

Receiver firmware for the GNSS modules (not the Pico firmware, which is in `firmware/`). Flash it with UPrecise on a Windows PC over the module's USB-C port (COM3). The Pico never flashes the receiver ([rp2350-feasibility.md](../../../plans/rp2350-feasibility.md)). What the builds mean for this project is in [gnss-um98x-config.md](../../../plans/gnss-um98x-config.md#receiver-firmware-builds).

## Source

Copied on 2026-11-12 from https://github.com/AlksSAV/gnss-firmware (`Firmware/Unicore/`, last push 2026-10-18). That repo has no license. The firmware is Unicore's and falls under Unicore's terms. Only the latest build per model is kept here. The source repo also has UM980 R4.10Build17548 and R4.10Build20739, and UM981-02 builds (AUTO GSP10598, SLANT GSP11767).

| File | Bytes | SHA-256 |
|---|---|---|
| `um980/UM980_R4.10Build25621.pkg` | 3004000 | `06b46b235f12a2f4d4410b28b1f8b1f5a3b12653cb1a988f1aa502ef5fbfc6d8` |
| `um982/UM982_R4.10Build18975.pkg` | 3027628 | `9ac25d1d359e5815b13291f61405faef53d71e306a602184546b32584b35ec71` |

The release notes are kept as they came: a Russian change list for the UM980 (converted from Windows-1251 to UTF-8) and Unicore's Chinese release note as an image for the UM982. The English translations below are ours.

## UM980 R4.10Build25621

No release date in the source.

1. Adapted to the new BeiDou PRN numbering. BeiDou satellite tracking strategy adjusted.
2. Encoding and decoding of Unicore's own (proprietary) differential corrections, which need less data than standard ones.
3. High-reliability mode: more robust ambiguity fixing and better fixed-solution accuracy.
4. B2b-PPP: changed how the B2b-PPP observations are used, fixing B2b-PPP, which stopped working after the BeiDou PRN rework.
5. L6 MADOCA-PPP: corrections applied in a better way, so the mode is more stable.
6. Anti-spoofing: spoofed signals are detected and a warning message is output.
7. QZSS L1C/B acquisition and tracking, and positioning with the QZSS L1S (SLAS) signal.
8. RTK library updated to cope with the BeiDou PRN rework and the retirement of the BeiDou-2 satellites.

## UM982 R4.10Build18975

Release date 2025-04-30, firmware upgrade. Our bench UM982 already runs this build.

1. Better ionosphere handling, and support for the TruePoint ionosphere quality-factor service. Enable with `CONFIG TPIONUSE ENABLE`.
2. QZSS L6 MADOCA-PPP. Enable with `CONFIG PPP ENABLE L6MDCPPP`.
3. PPP fusion: with PPP in auto mode, QZSS L6 MADOCA-PPP corrections are fused in. Shorter convergence and better accuracy, mainly in open sky.
4. RTK-assisted PPP: once RTK is fixed, PPP converges to cm level within 20 s even under light obstruction, and keeps cm level for more than a minute after RTK is lost.
5. Configurable PVT reliability threshold, which sets how reliable single-point position and Doppler velocity must be.
6. Smoother best-position output in difficult conditions, with far fewer position jumps when the position type changes.
7. Anti-jamming channel setup changed for better interference rejection on the master antenna.
8. GSV output follows the NMEA 0183 v4.11 errata: the last field is now the **signal ID** instead of the system ID (as in v4.10).

The release came with an update to the Unicore N4 High Precision Products reference commands manual, Chinese V2 R1.8. The English R1.16 in `docs/` is newer.
