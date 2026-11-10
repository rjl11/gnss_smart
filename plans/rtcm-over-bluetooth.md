# RTCM corrections from SW Maps (Android) over Bluetooth

## Context
The external correction radio (formerly on UM980 COM2) has been dropped. RTK corrections now come from **SW Maps** on an Android phone:
- SW Maps connects to the device as a Bluetooth GNSS receiver (a "Generic NMEA" device, over Bluetooth LE or Classic) and reads our NMEA.
- SW Maps runs its own NTRIP client, using the phone's mobile data.
- It sends the RTCM it receives back to the device over the same Bluetooth link.

This plan covers that link on the Pico 2 W (CYW43439 + BTstack) and the path from Bluetooth into the UM980. The NMEA output side was already planned in [rp2350-feasibility.md](rp2350-feasibility.md).

## One Bluetooth link, both directions
| Direction | Content | Rate |
|---|---|---|
| Device → phone | NMEA: GGA and RMC at 10 Hz, plus `$GNHDT` on a UM982 ([gnss-um982-dual-antenna.md](gnss-um982-dual-antenna.md)) and GST/GSA if cheap | ~1.5–2 kB/s |
| Phone → device | RTCM3 from the NTRIP caster | ~1–3 kB/s (MSM4/MSM7, 4 constellations, 1 Hz) |

Both fit easily in BLE throughput with a 247-byte MTU. RTCM arrives in arbitrary chunks that don't line up with frame boundaries, so the firmware has to reassemble the frames.

## GGA to the caster
- SW Maps forwards our GGA to the caster when "Send NMEA GGA to Caster" is on. Network/VRS casters need this; without GGA the receiver stays in float.
- So **GGA must carry a real position as soon as the UM980 has a fix**, even before the INS is navigating. Until then, fall back to the UM980's own position.
- SW Maps throttles how often it sends GGA to the caster, so our 10 Hz rate is fine.

## Bluetooth profile
- **First choice: BLE with the Nordic UART Service (NUS)** via BTstack (`pico_btstack_ble`), as already planned for the output.
  - The TX characteristic (notify) carries NMEA out.
  - The RX characteristic (write without response) carries RTCM in.
  - **To verify:** that SW Maps' "Generic NMEA (Bluetooth LE)" driver finds a NUS device and writes RTCM to its RX characteristic. SW Maps' docs don't confirm this.
- **Fallback: Bluetooth Classic SPP (Serial Port Profile).**
  - SW Maps also supports Classic Bluetooth GNSS receivers, and this is the most common way Bluetooth RTK receivers work with it.
  - The CYW43439 and BTstack support dual mode, so the Pico 2 W can offer SPP.
  - Use it if SW Maps' BLE path doesn't send RTCM. The config page chooses between BLE and SPP.
  - An ESP32-S3 has no Bluetooth Classic, so this fallback doesn't exist there ([esp32-s3-feasibility.md](esp32-s3-feasibility.md)).

## Path into the UM980
1. **BTstack receive callback (core 0) → ring buffer** (4 KB is several seconds of RTCM).
2. **RTCM3 frame check:**
   - Each frame is a 0xD3 preamble, a 10-bit length and a CRC-24Q.
   - Forward only whole frames that pass the CRC. Count and drop bad ones.
   - Note the message type of each frame for diagnostics.
3. **UART1 TX → UM980 COM1 at 460800**, the same port that carries commands and binary logs.
   - The `um980_tx` arbiter owns UART1 TX. It writes only whole RTCM frames or whole command lines, so the two never interleave. Details in [gnss-um980-config.md](gnss-um980-config.md).
   - **No port is locked.** `CONFIG RXTYPE` is never sent, because of the risk of bricking. COM1 stays at the default `AUTO`, and in rover mode the receiver recognises RTCM by itself.
   - Only one cable is needed, and no PIO state machine. COM3 is the module's USB-C port, left for a PC, and the COM2 pins are not connected.
- **Rejected alternative:** a PIO UART TX to COM2, locked with `CONFIG RXTYPE COM2 RTCM`. It keeps COM1 free of corrections, but it needs a port lock (bricking risk) and a second wire.
- **Zero 2 W companion variant:** RTCM goes from the Zero's UART into COM2, but with **no** port lock. COM2 stays `AUTO` and the Zero writes only CRC-checked RTCM frames ([pico2-zero2w-usb-companion.md](pico2-zero2w-usb-companion.md)). The rejected variant above needed `RXTYPE`; this one doesn't.

**Reference: how the AiO v26 firmware forwards RTCM.** The AgOpenGPS AiO v26 Teensy firmware (https://github.com/AgOpenGPS-Official/Firmware_Teensy_AiO_26, `lib/aio_system/RTCMProcessor.cpp` and `QNEthernetUDPHandler.cpp`, commit `2551bff`, read 2026-11-10, GPL v3) takes RTCM from UDP port 2233 or a radio and writes it to the receiver's UART at 460800. Facts from its code; no code is taken.
- **No frame check:** it writes each datagram to the UART as raw bytes. Its only test is a length of at least 5 bytes. Our CRC-24Q check is stricter, and it is what lets commands and corrections share COM1.
- **A field bug it fixed** (its issue #32, commit `bfa824f`): corrections were lost because the receive queue held one datagram, and a second one arriving before the next poll replaced the first. It now queues 8 datagrams.
- **Never block on the UART:** since that fix it forwards one datagram per loop pass, and only when the UART TX buffer has at least 256 bytes free, so the write can't stall the main loop. The same rule applies to `um980_tx`: write a frame only when it fits, and let a burst wait in the ring buffer (4 KB here).

## Monitoring and failure behaviour
- **Correction age and RTK status** come from BESTNAVB (differential age, position type), which the UM980 parser already reads.
- **If Bluetooth drops:**
  1. Corrections stop.
  2. After its correction-age timeout, the UM980 drops to float, then single.
  3. The fix quality in our GGA follows automatically (and in `$PAOGI`, once Ethernet is back from on hold).
  4. The filter keeps running.
- **Diagnostics page** ([web-config-server.md](web-config-server.md)):
  - Bluetooth connected, and whether it's BLE or SPP.
  - RTCM bytes and frames received, CRC errors, time since the last frame.
  - Message types seen, e.g. 1005, 1074/1077, 1084/1087, 1094/1097, 1124/1127, 1230.
  - Correction age as reported by the UM980.

## Timing
Everything runs on core 0, as laid out in [rtos-decision.md](rtos-decision.md). The filter on core 1 is unaffected. Bluetooth receive load is small (1–3 kB/s, in chunks of up to a few hundred bytes).

## Field notes
- The phone must stay within Bluetooth range, so mount it in the cab, not in a pocket outside.
- The phone needs mobile data for NTRIP.
- Android battery optimisation can stop SW Maps in the background. Keep it in the foreground, or exempt it from battery optimisation.
- Only one app can hold the Bluetooth link. SW Maps must be both the NMEA reader and the NTRIP client; a second NTRIP app can't share the connection.

## Verification (when implementing)
1. SW Maps connects over BLE as a Generic NMEA device and shows the device's position.
2. With the NTRIP client on, RTCM frames are counted on the diagnostics page with zero CRC errors.
3. The UM980 reaches RTK fixed, and correction age stays under 5 s.
4. Turn Bluetooth off on the phone. Correction age rises, the fix drops to float and then single, and core 1 filter timing (bench replay from [rp2350-feasibility.md](rp2350-feasibility.md)) doesn't change.
5. If step 1 or 2 fails over BLE, repeat everything over Classic SPP.
