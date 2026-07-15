# Rivaler Pin Map

This is the initial wiring map for firmware development. Inputs marked
`INPUT_PULLUP` connect to ground when activated. All boards sharing a signal
must share ground, but their power rails remain separate as shown in the
connection diagram.

## Remote Nano ESP32

| Pin | Connection |
| --- | --- |
| A0 | Joystick X |
| A1 | Joystick Y |
| D2 | Quick Shoot button |
| D3 | Rapid Fire button |
| D4 | Take Picture button |
| D5 | Eject SD button |
| D6 | Joystick push / Follow Line button |
| D7 | Launcher arm switch |
| D12 | Wireless software emergency-stop button |
| D8 | Active buzzer signal |
| D9 | Launcher Armed LED |
| D10 | Line Detected LED |
| D11 | Line Following LED |

The Remote Nano uses ESP-NOW with Robot Nano MAC
`3C:84:27:FC:EF:2C`. Its own MAC is `20:6E:F1:32:60:BC`.

The remote uses five discrete buttons: Quick Shoot, Rapid Fire, Take Picture,
Eject SD, and wireless emergency stop. The joystick push switch controls
Follow Line, and the separate switch controls launcher arming.

## Robot Nano ESP32

| Pin | Connection |
| --- | --- |
| D6 | UART RX from GIGA D14 / Serial4 TX3 |
| D7 | UART TX to GIGA D15 / Serial4 RX3 |
| D8 | UART TX to Mega D19 / RX1 |

The Mega-to-Nano UART return wire is intentionally absent.

## Arduino GIGA

| Pins | Connection |
| --- | --- |
| D3, D6 | Shield channel A: enable, speed |
| D4, D5 | Shield channel B: enable, speed |
| D7, D10 | Shield channel C: enable, speed |
| D8, D9 | Shield channel D: enable, speed |
| D14, D15 | Serial4 TX/RX for Robot Nano (board labels TX3/RX3), dedicated UART exception |
| A0, A1 | Left and right line sensors |
| D22, D23 | Front ultrasonic trigger/echo |
| D24, D25 | Downward ultrasonic trigger/echo |
| D26, D27 | Left ultrasonic trigger/echo |
| D28, D29 | Right ultrasonic trigger/echo |
| D30, D31 | Rear ultrasonic trigger/echo |
| D32 | 360-degree positional launcher magazine servo; homes at 0 degrees and indexes 45, 90, 135, 180, -45, -90, -135, then 0 |
| D33, D34, D35 | External launcher H-bridge: IN1, IN2, full-speed enable |

Connect the left bipolar stepper to shield channels A and B, and the right
bipolar stepper to channels C and D. The shield has no remaining H-bridge
channel for the launcher DC motor. The external H-bridge drives that motor
from D33, D34, and D35 instead.

The project's HC-SR04 modules have been verified operating at 3.3 V with the
GIGA, so their Echo signals connect directly to D23, D25, D27, D29, or D31.
The servo signal is D32, but servo power must come from its external supply
with a common ground to the GIGA.

## Arduino Mega

| Pins | Connection |
| --- | --- |
| D19 | UART RX from Robot Nano D8 |
| ICSP: 50/51/52 | Arducam Mega B0401 SPI: MISO/MOSI/SCK |
| D7 | Arducam B0401 chip select |
| ICSP: 50/51/52 | SD module shared SPI bus |
| D4 | SD module chip select |

The Robot Nano's 3.3 V TX signal is acceptable at Mega D19. Do not wire the
Mega's 5 V TX pin back to either Nano. Keep the inactive SPI device's chip
select HIGH to prevent camera/SD bus contention.
