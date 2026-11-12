# ToDo

Open work items. The details are in the linked plans.

- [ ] **UM982: find the cause of the 1 s BESTNAVB gap.** At 20 Hz, BESTNAVB sometimes skips exactly one second (about 20 epochs). It happened about once a minute in half the runs, on `3 6` and `7 0`, with any log set (bench results 2026-11-12).
  - **Setup:** the UM982 on miniPi (`/dev/ttyUSB0`), COM3 set to 115200 without `SAVECONFIG`, because miniPi drops bytes at 460800. `RESET` afterwards to get back the saved 460800.
  - **Test:** log `BESTNAVB COM3 0.05`, `STADOPB COM3 1` and `HWSTATUSB COM3 1` for 10 min. Record every gap in the BESTNAVB header times, with its GPS time.
  - **Look for:**
    - whether the gaps fall on whole GPS seconds or come at a fixed interval;
    - whether they line up with HWSTATUS or STADOP output;
    - whether the receiver or the USB link drops the epochs. Check the header's CPU idle byte and the CRC errors, and run a raw `cat` capture alongside.
  - See [plans/gnss-um98x-config.md](plans/gnss-um98x-config.md), section "Bench results (2026-11-12, UM982 on `3 6`: navigation rate)", and its open question.
