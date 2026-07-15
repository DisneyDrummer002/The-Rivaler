# The Rivaler

The Rivaler is a multi-controller Arduino robot project with wireless remote control, stepper drive, line following, ultrasonic safety checks, a launcher mechanism, and camera/SD-card control.

> [!CAUTION]
> This project controls moving machinery and a launcher. Read the [safety requirements](docs/safety.md) before powering motors. The wireless software stop is not a substitute for a normally closed, latching physical emergency-stop circuit that removes motor power.

## Repository contents

| Path | Contents |
| --- | --- |
| [`firmware/giga_main_controller`](firmware/giga_main_controller) | Arduino GIGA drive, launcher, line-following, and ultrasonic-safety firmware |
| [`firmware/robot_nano_esp32`](firmware/robot_nano_esp32) | Robot-side Nano ESP32 wireless/UART bridge |
| [`firmware/remote_nano_esp32`](firmware/remote_nano_esp32) | Handheld remote firmware |
| [`firmware/mega_camera_controller`](firmware/mega_camera_controller) | Arduino Mega camera and SD-card controller |
| [`firmware/giga_stepper_test`](firmware/giga_stepper_test) | Standalone GIGA stepper-motor test sketch |
| [`firmware/shared`](firmware/shared) | Shared packet protocol and host-side protocol test |
| [`firmware/libraries/RivalerProtocol`](firmware/libraries/RivalerProtocol) | Arduino library wrapper for the shared protocol |
| [`mactest`](mactest) | Nano ESP32 utility sketch for printing a board's Wi-Fi MAC address |
| [`docs`](docs) | Wiring, protocol, line-following, camera, and safety documentation |
| [`BoM.xlsx`](BoM.xlsx) | Bill of materials |
| [`The Rivaler Connections.drawio.svg`](The%20Rivaler%20Connections.drawio.svg) | Editable Draw.io connection diagram in SVG form |
| [`Prompt Document for The Rivaler.docx`](Prompt%20Document%20for%20The%20Rivaler.docx) | Project prompt and design document |

## Hardware targets

- Arduino GIGA R1 WiFi: main controller and stepper test
- Two Arduino Nano ESP32 boards: remote and robot bridge
- Arduino Mega 2560: Arducam Mega B0401 and SD-card controller

See the [pin map](docs/pin-map.md) and the Draw.io connection diagram before wiring the boards.

## Build notes

Install [Arduino CLI](https://arduino.github.io/arduino-cli/) or use Arduino IDE 2. Install the Arduino Mbed OS GIGA Boards and Arduino ESP32 Boards platforms. The camera controller also requires the official `Arducam_Mega` library described in [the camera documentation](docs/mega-camera.md).

Example Arduino CLI commands for the sketches that do not require the camera library:

```sh
arduino-cli compile --fqbn arduino:mbed_giga:giga firmware/giga_main_controller
arduino-cli compile --fqbn arduino:mbed_giga:giga firmware/giga_stepper_test
arduino-cli compile --fqbn arduino:esp32:nano_nora firmware/robot_nano_esp32
arduino-cli compile --fqbn arduino:esp32:nano_nora firmware/remote_nano_esp32
```

Before uploading, review each sketch's `Config.h`, calibrate the line sensors as documented in [line following](docs/line-following.md), and verify the board addresses and physical wiring.

## Documentation

- [Pin map](docs/pin-map.md)
- [Packet protocol](docs/protocol.md)
- [Line following and calibration](docs/line-following.md)
- [Mega camera and SD card](docs/mega-camera.md)
- [Safety requirements](docs/safety.md)

## Project status

This is active hardware-development firmware. Test with drive wheels raised and launcher motor power disconnected before performing controlled, low-speed floor tests.
