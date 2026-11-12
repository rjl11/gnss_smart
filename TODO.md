# ToDo

Open work items. The details are in the linked plans.

- [ ] **UM980 / UM982: find the cause of the 1 s BESTNAVB gap.** At 20 Hz, BESTNAVB sometimes skips exactly one second (about 20 epochs). It happened about once a minute in half the runs, on the UM982 (`3 6` and `7 0`) and on the UM980 (SG 2), with any log set (bench results 2026-11-12). Both were read through the same miniPi and USB cable, so the host side is a suspect too.
  - **Setup:** the receiver on miniPi (`/dev/ttyUSB0`), COM3 at 115200. A receiver saved at 460800 gets COM3 set to 115200 without `SAVECONFIG`, because miniPi drops bytes at 460800, and a `RESET` afterwards. If miniPi is ruled out, repeat on the Windows PC.
  - **Test:** log `BESTNAVB COM3 0.05`, `STADOPB COM3 1` and `HWSTATUSB COM3 1` for 10 min. Record every gap in the BESTNAVB header times, with its GPS time.
  - **Look for:**
    - whether the gaps fall on whole GPS seconds or come at a fixed interval;
    - whether they line up with HWSTATUS or STADOP output;
    - whether the receiver or the USB link drops the epochs. Check the header's CPU idle byte and the CRC errors, and run a raw `cat` capture alongside.
  - See [plans/gnss-um98x-config.md](plans/gnss-um98x-config.md), sections "Bench results (2026-11-12, UM982 on `3 6`: navigation rate)" and "Bench results (2026-11-12, UM980 on SG 2: navigation rate)", and the open question.
