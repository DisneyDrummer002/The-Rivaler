#pragma once

#include <stdint.h>

namespace rivaler_remote_config {

// ESP-NOW peer identity. The remote sends to the Robot Nano and receives status
// from it over the same peer relationship.
constexpr uint8_t kRobotNanoMac[6] = {0x3C, 0x84, 0x27, 0xFC, 0xEF, 0x2C};
constexpr uint8_t kEspNowChannel = 0;  // Use the current Wi-Fi channel.

constexpr unsigned long kEspNowStatusTimeoutMs = 1500;
constexpr unsigned long kControlTransmitIntervalMs = 50;

// Joystick axes are analog inputs. Center values are conservative starting
// points for the Nano ESP32's 12-bit ADC and should be calibrated at bring-up.
constexpr int kJoystickXPin = A0;
constexpr int kJoystickYPin = A1;
constexpr int kJoystickAdcMaximum = 4095;
constexpr int kJoystickXCenter = 2048;
constexpr int kJoystickYCenter = 2048;
constexpr int kJoystickDeadZone = 160;
constexpr bool kJoystickYPositiveIsForward = true;

// All switches and buttons connect to ground when active and use INPUT_PULLUP.
constexpr int kQuickShootButtonPin = D2;
constexpr int kRapidFireButtonPin = D3;
constexpr int kTakePictureButtonPin = D4;
constexpr int kEjectSdButtonPin = D5;
constexpr int kFollowLineButtonPin = D6;  // Joystick push switch.
constexpr int kArmSwitchPin = D7;
constexpr int kEmergencyStopButtonPin = D12;
constexpr bool kInputsAreActiveLow = true;
constexpr unsigned long kDebounceMs = 30;

constexpr int kBuzzerPin = D8;
constexpr int kLauncherArmedLedPin = D9;
constexpr int kLineDetectedLedPin = D10;
constexpr int kLineFollowingLedPin = D11;
constexpr unsigned long kSafetyBuzzerOnMs = 100;
constexpr unsigned long kSafetyBuzzerOffMs = 400;
constexpr unsigned long kLinkLostBuzzerOnMs = 60;
constexpr unsigned long kLinkLostBuzzerOffMs = 940;

}  // namespace rivaler_remote_config
