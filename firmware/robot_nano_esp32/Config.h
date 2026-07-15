#pragma once

#include <stdint.h>

namespace rivaler_robot_nano_config {

constexpr uint8_t kRemoteNanoMac[6] = {0x20, 0x6E, 0xF1, 0x32, 0x60, 0xBC};
constexpr uint8_t kEspNowChannel = 0;  // Use the current Wi-Fi channel.

constexpr unsigned long kRemoteCommandTimeoutMs = 300;
constexpr unsigned long kGigaStatusTimeoutMs = 500;

// ESP32 hardware UART ports are assigned in the bridge sketch. These GPIOs
// are crossed to the indicated receiver pins on the other boards.
constexpr int kGigaUartPort = 1;
constexpr int kGigaUartRxPin = D6;  // From GIGA D14 / Serial4 TX3.
constexpr int kGigaUartTxPin = D7;  // To GIGA D15 / Serial4 RX3.
constexpr unsigned long kGigaUartBaud = 115200;

constexpr int kMegaUartPort = 2;
constexpr int kMegaUartTxPin = D8;  // To Mega D19 / RX1 only.
constexpr unsigned long kMegaUartBaud = 115200;

constexpr uint8_t kQuickShootPhotoCount = 1;
constexpr uint8_t kRapidFirePhotoCount = 8;
constexpr unsigned long kRapidFirePhotoIntervalMs = 3000;
constexpr uint8_t kManualCapturePhotoCount = 10;
constexpr unsigned long kManualCapturePhotoIntervalMs = 500;

}  // namespace rivaler_robot_nano_config
