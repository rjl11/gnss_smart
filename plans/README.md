# Plans

Planning documents for gnss_smart: system architecture, hardware selection (MCU, sensors), sensor fusion design and roadmap.

| Plan | Topic |
|---|---|
| [rp2350-feasibility.md](rp2350-feasibility.md) | Pico 2 W / RP2350 CPU budget, serial port layout, Bluetooth output |
| [esp32-p4-feasibility.md](esp32-p4-feasibility.md) | ESP32-P4 as an alternative MCU |
| [inslib-nav-suite.md](inslib-nav-suite.md) | INSLIB integration through `nav_suite`: options, epoch building, outputs |
| [imu-tm210.md](imu-tm210.md) | TM210 IMU configuration, protocol, unit/frame conversion and calibration for INSLIB |
| [rtos-decision.md](rtos-decision.md) | Bare metal first, FreeRTOS later |
| [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md) | W5500 Ethernet, `$PANDA` output for AgOpenWeb, RTCM in |
| [web-config-server.md](web-config-server.md) | GPIO-toggled web config and diagnostics server |
