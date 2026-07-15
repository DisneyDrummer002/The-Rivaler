# Rivaler Packet Protocol

Protocol version 1 is the shared binary format for both ESP-NOW and UART.
It keeps the remote, bridge, GIGA, and camera controller aligned without
depending on ad-hoc text parsing.

## Packet layout

| Byte(s) | Field | Description |
| --- | --- | --- |
| 0 | Start | `0xA5` |
| 1 | Version | `1` |
| 2 | Type | Message type identifier |
| 3 | Sequence | Sender-local sequence; event retransmissions reuse their original value |
| 4 | Payload length | `0` through `24` |
| 5... | Payload | Binary payload for the packet type |
| Final | Checksum | XOR of bytes 1 through the final payload byte |

Only `6 + payload length` bytes are sent. Every receiver validates start byte,
version, exact received length, payload length, and checksum before acting.

## Transport routes

| Route | Direction | Message types |
| --- | --- | --- |
| Remote Nano ESP32 and Robot Nano ESP32 | Two-way ESP-NOW | Remote control, robot status, acknowledgement |
| Robot Nano ESP32 and Arduino GIGA | Two-way UART | Remote control, robot status, acknowledgement |
| Robot Nano ESP32 to Arduino Mega | One-way UART | Camera command |

The Remote Nano (`20:6E:F1:32:60:BC`) and Robot Nano
(`3C:84:27:FC:EF:2C`) are paired peers. ESP-NOW traffic is bidirectional;
the labels only identify each board's MAC address.

## Message payloads

### `kRemoteControl` (`0x10`)

| Field | Type | Meaning |
| --- | --- | --- |
| `joystickX` | signed 8-bit | `-100` to `100`; left/right command after dead-zone filtering |
| `joystickY` | signed 8-bit | `-100` to `100`; positive is forward |
| `heldButtons` | unsigned 16-bit flags | Current state of arm switch, line-follow button, and emergency stop |
| `edgeEvents` | unsigned 16-bit flags | One-time button presses: quick shoot, rapid fire, take picture, eject SD, line-follow press |

`heldButtons` is sent with every control packet. For packets containing
`edgeEvents`, the Remote Nano retransmits the same sequence until the Robot
Nano acknowledges receipt. The Robot Nano suppresses duplicate sequences, so
retransmission cannot stack launcher or camera actions.

### `kRobotStatus` (`0x20`)

| Field | Type | Meaning |
| --- | --- | --- |
| `statusFlags` | unsigned 16-bit flags | Launcher, line, safety, camera, and SD state |
| `frontDistanceCm` | unsigned 16-bit | Filtered front ultrasonic reading; `0` means unavailable |
| `groundDistanceCm` | unsigned 16-bit | Filtered downward sensor reading; `0` means unavailable |
| `safetyFault` | unsigned 8-bit | Current fail-safe reason, or `kNone` |

### `kCameraCommand` (`0x30`)

| Field | Type | Meaning |
| --- | --- | --- |
| `operation` | unsigned 8-bit enum | Capture burst or prepare SD eject |
| `photoCount` | unsigned 8-bit | Number of photos for a capture burst |
| `intervalMs` | unsigned 16-bit | Delay between photos |

The Robot Nano creates camera commands after receiving applicable remote
events. The Arduino Mega does not send command replies back over this link.

### `kAcknowledgement` (`0x40`)

| Field | Type | Meaning |
| --- | --- | --- |
| `acknowledgedSequence` | unsigned 8-bit | Sequence number being acknowledged |
| `accepted` | unsigned 8-bit | `1` when accepted, `0` when rejected |
| `rejectionReason` | unsigned 8-bit enum | `SafetyFault` value, otherwise `kNone` |

The Robot Nano acknowledges valid remote-control packets that contain one-shot
events. An acknowledgement means the bridge received the packet; it is not
permission to keep moving. The GIGA still stops drive motion and disarms the
launcher when valid remote control packets time out.

## Safety rules at the protocol boundary

- Receivers ignore malformed, stale, unknown-version, or checksum-invalid packets.
- A receiver must never fire or arm based on an incomplete packet.
- The Robot Nano forwards only valid remote-control packets to the GIGA.
- The GIGA is the final authority for drive, safety, arming, and any launcher action.
- The Mega accepts only valid camera-command packets and has no path to control the GIGA.
