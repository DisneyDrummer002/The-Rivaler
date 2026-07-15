#include "MegaCameraCommand.h"

#include "Config.h"

using namespace rivaler_robot_nano_config;

MegaCameraCommand::MegaCameraCommand(HardwareSerial& serialPort)
    : serialPort_(serialPort) {}

void MegaCameraCommand::begin(unsigned long baud, int transmitPin) {
  serialPort_.begin(baud, SERIAL_8N1, -1, transmitPin);
}

void MegaCameraCommand::sendForRemoteControl(
    const rivaler::RemoteControlPayload& control, uint8_t& nextSequence) {
  const uint16_t events = control.edgeEvents;

  if ((events & rivaler::kEjectSdPressed) != 0) {
    sendPrepareSdEject(nextSequence);
    return;
  }

  if ((events & rivaler::kRapidFirePressed) != 0) {
    sendCaptureBurst(kRapidFirePhotoCount, kRapidFirePhotoIntervalMs,
                     nextSequence);
    return;
  }

  if ((events & rivaler::kQuickShootPressed) != 0) {
    sendCaptureBurst(kQuickShootPhotoCount, 0, nextSequence);
    return;
  }

  if ((events & rivaler::kTakePicturePressed) != 0) {
    sendCaptureBurst(kManualCapturePhotoCount, kManualCapturePhotoIntervalMs,
                     nextSequence);
  }
}

void MegaCameraCommand::sendCaptureBurst(uint8_t photoCount, uint16_t intervalMs,
                                         uint8_t& nextSequence) {
  rivaler::CameraCommandPayload command{};
  command.operation = rivaler::CameraOperation::kCaptureBurst;
  command.photoCount = photoCount;
  command.intervalMs = intervalMs;

  rivaler::Packet packet{};
  rivaler::initializePacket(packet, rivaler::MessageType::kCameraCommand,
                            nextSequence++);
  if (rivaler::setPayload(packet, command)) {
    sendPacket(packet);
  }
}

void MegaCameraCommand::sendPrepareSdEject(uint8_t& nextSequence) {
  rivaler::CameraCommandPayload command{};
  command.operation = rivaler::CameraOperation::kPrepareSdEject;

  rivaler::Packet packet{};
  rivaler::initializePacket(packet, rivaler::MessageType::kCameraCommand,
                            nextSequence++);
  if (rivaler::setPayload(packet, command)) {
    sendPacket(packet);
  }
}

bool MegaCameraCommand::sendPacket(const rivaler::Packet& packet) {
  uint8_t wireBytes[rivaler::kMaxPacketBytes]{};
  const uint8_t wireSize = rivaler::packetWireSize(packet);
  if (!rivaler::encodePacket(packet, wireBytes, sizeof(wireBytes))) {
    return false;
  }

  return serialPort_.write(wireBytes, wireSize) == wireSize;
}
