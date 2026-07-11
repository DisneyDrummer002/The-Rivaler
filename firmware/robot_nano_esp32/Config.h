#pragma once

#include <stdint.h>

namespace rivaler_robot_nano_config {

constexpr uint8_t kRemoteNanoMac[6] = {0x3C, 0x84, 0x27, 0xFC, 0xEF, 0x2C};
constexpr uint8_t kEspNowChannel = 0;  // Use the current Wi-Fi channel.

constexpr unsigned long kRemoteCommandTimeoutMs = 300;
constexpr unsigned long kGigaStatusTimeoutMs = 500;

// ESP32 hardware UART ports are assigned in the bridge sketch. These GPIOs
// are crossed to the indicated receiver pins on the other boards.
constexpr int kGigaUartPort = 1;
constexpr int kGigaUartRxPin = 6;  // From GIGA D14 / TX3.
constexpr int kGigaUartTxPin = 7;  // To GIGA D15 / RX3.
constexpr unsigned long kGigaUartBaud = 115200;

constexpr int kMegaUartPort = 2;
constexpr int kMegaUartTxPin = 8;  // To Mega D19 / RX1 only.
constexpr unsigned long kMegaUartBaud = 115200;

constexpr uint8_t kQuickShootPhotoCount = 1;
constexpr uint8_t kRapidFirePhotoCount = 8;
constexpr unsigned long kRapidFirePhotoIntervalMs = 3000;
constexpr uint8_t kManualCapturePhotoCount = 10;
constexpr unsigned long kManualCapturePhotoIntervalMs = 500;

}  // namespace rivaler_robot_nano_config
