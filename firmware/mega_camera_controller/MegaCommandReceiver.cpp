#include "MegaCommandReceiver.h"

#include "Config.h"

using namespace rivaler_mega_config;

MegaCommandReceiver::MegaCommandReceiver(HardwareSerial& serialPort)
    : serialPort_(serialPort) {}

void MegaCommandReceiver::begin(unsigned long baud) { serialPort_.begin(baud); }

bool MegaCommandReceiver::readCommand(rivaler::CameraCommandPayload& command) {
  constexpr uint8_t kMaximumBytesPerPoll = 64;
  uint8_t processedBytes = 0;
  rivaler::Packet packet{};

  while (serialPort_.available() > 0 && processedBytes < kMaximumBytesPerPoll) {
    ++processedBytes;
    if (!parser_.pushByte(static_cast<uint8_t>(serialPort_.read()), packet)) {
      continue;
    }

    if (packet.type != rivaler::MessageType::kCameraCommand ||
        !rivaler::readPayload(packet, command) || !isCommandValid(command)) {
      continue;
    }

    return true;
  }

  return false;
}

bool MegaCommandReceiver::isCommandValid(
    const rivaler::CameraCommandPayload& command) const {
  if (command.operation == rivaler::CameraOperation::kPrepareSdEject) {
    return command.photoCount == 0 && command.intervalMs == 0;
  }

  if (command.operation != rivaler::CameraOperation::kCaptureBurst) {
    return false;
  }

  return command.photoCount > 0 && command.photoCount <= kMaximumPhotoCount &&
         command.intervalMs <= kMaximumPhotoIntervalMs;
}
