# Plans

Planning documents for gnss_smart: system architecture, hardware selection (MCU, sensors), sensor fusion design and roadmap.

| Plan | Topic |
|---|---|
| [rp2350-feasibility.md](rp2350-feasibility.md) | Pico 2 W / RP2350 CPU budget, serial port layout, Bluetooth output |
| [esp32-p4-feasibility.md](esp32-p4-feasibility.md) | ESP32-P4 as an alternative MCU |
| [esp32-s3-feasibility.md](esp32-s3-feasibility.md) | ESP32-S3 as an alternative MCU: CPU budget, UART count, Wi-Fi/BLE, risks |
| [esp32-s3-wifi-companion.md](esp32-s3-wifi-companion.md) | Plain Pico 2 + ESP32-S3 as Wi-Fi/BLE companion: link, protocol, split of work |
| [inslib-nav-suite.md](inslib-nav-suite.md) | INSLIB integration through `nav_suite`: options, epoch building, outputs |
| [imu-tm210.md](imu-tm210.md) | TM210 IMU configuration, protocol, unit/frame conversion and calibration for INSLIB |
| [imu-adis16465-vs-tm210.md](imu-adis16465-vs-tm210.md) | ADIS16465-2 as an alternative IMU: datasheet comparison, system effect, firmware impact |
| [gnss-mosaic-x5-vs-um980.md](gnss-mosaic-x5-vs-um980.md) | Septentrio mosaic-X5 as an alternative GNSS receiver: datasheet comparison, system effect, firmware impact |
| [rtos-decision.md](rtos-decision.md) | Bare metal first, FreeRTOS later |
| [ethernet-w5500-agopenweb.md](ethernet-w5500-agopenweb.md) | W5500 Ethernet, `$PANDA` output for AgOpenWeb, RTCM in |
| [web-config-server.md](web-config-server.md) | GPIO-toggled web config and diagnostics server |
