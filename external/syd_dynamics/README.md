# SYD Dynamics TransducerM material

This folder holds the TransducerM example projects and the TM210 3D model from SYD Dynamics, as supplied with the product. SYD publishes no repository, so this is a plain vendored copy, not a git submodule. The files are unmodified; only the ROS package's local `.git` history was removed.

**None of this is compiled into the firmware.** The firmware has its own small C codec for EasyProtocol. This folder is the reference for it; see [plans/imu-tm210-config.md](../../plans/imu-tm210-config.md) and [plans/imu-tm210.md](../../plans/imu-tm210.md).

## Contents
| Item | What it is |
|---|---|
| `TransducerM_Example_CPP_QT_V131-R1/` | Qt desktop example with EasyProfile **V1-3-1-R1** (2025). This is the **reference copy for the TM210 settings**: the only library copy that defines `Ep_Settings` / `Ep_Switches`. `HelloMotionModule.cpp` `Example_ChangeSetting()` shows the settings write sequence (request, modify, send to the node ID, wait for the Ack) |
| `TransducerM_Example_RaspberryPiPico_V1-1/` | Arduino-on-RP2040 example: UART0 on GP0/GP1 at 115200, calibration commands |
| `TransducerM_Example_STM32_CPP/` | STM32F4 HAL example, receives with `HAL_UARTEx_ReceiveToIdle_DMA`. Includes ST-generated files |
| `TransducerM_Example_Arduino_V1-4/` | Arduino example. `libraries/EasyProfile.zip` is the same library as the unpacked folder |
| `TransducerM_Example_ROS-pkg_V1-9/` | ROS 1 package, uses `serialib` |
| `TransducerM_Example_ROS2-pkg_V1-1/` | ROS 2 node with rviz screenshots, uses `serialib` |
| `TransducerM_Protocol_C_V1-2-1-Beta/` | Plain C version of the library (2017). Request and receive only |
| `TransducerM_TM210_3D-Shape_V1-1-2.stp` | TM210 STEP model, for mounting and enclosure design |

None of these examples write the output data rate (ODR), and none show how ODR is encoded in the Setting object. That still has to be read off a real TM210 (see the verification section of the TM210 config plan).

## Known differences between the library copies
- **The 3 header bits after CMD:** the embedded and ROS copies call them `qos` ("Quality-of-Service of the source device (only available on certain Models)"). The QT copy and the user guide call them `res`, "must be zero". No copy's parser rejects non-zero values. The firmware accepts any value and reports it as QoS.
- **CRC write:** the STM32 copy's `EasyProtocol.cpp` writes the CRC byte by byte. The QT copy does a single `uint16` store, which can be unaligned.
- **`Ep_Settings` / `Ep_Calib`:** only in the QT copy. The embedded copies leave them out.

## Licenses
- EasyProfile: BSD-style, © SYD Dynamics ApS. The notice is in each library's `License.txt` and file headers, and must be kept.
- STM32 project: ST's generated files carry ST's own license headers.
- `serialib` (ROS packages): its own headers.
