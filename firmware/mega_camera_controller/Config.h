#pragma once

#include <stdint.h>

namespace rivaler_mega_config {

constexpr int kUnassignedPin = -1;

// One-way Robot Nano to Mega command link. Mega TX remains disconnected.
constexpr int kRobotNanoCommandRxPin = 19;  // Mega Serial1 RX.
constexpr int kRobotNanoCommandTxPin = kUnassignedPin;
constexpr unsigned long kRobotNanoCommandBaud = 115200;

// Arducam Mega B0401 is a 5 MP SPI camera. Use the Mega's hardware SPI pins
// on the ICSP header (MISO 50, MOSI 51, SCK 52) and this dedicated chip select.
constexpr char kCameraSku[] = "B0401";
constexpr int kCameraChipSelectPin = 7;

// The SD module shares the Mega's SPI bus. Its chip select must remain HIGH
// whenever the camera is active, and vice versa.
constexpr int kSdChipSelectPin = 4;

constexpr uint8_t kDefaultPhotoCount = 10;
constexpr unsigned long kDefaultPhotoIntervalMs = 500;
constexpr uint8_t kMaximumPhotoCount = 10;
constexpr unsigned long kMaximumPhotoIntervalMs = 5000;

}  // namespace rivaler_mega_config
