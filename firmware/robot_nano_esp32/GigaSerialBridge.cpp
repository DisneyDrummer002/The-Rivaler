#include "GigaSerialBridge.h"

GigaSerialBridge::GigaSerialBridge(HardwareSerial& serialPort)
    : serialPort_(serialPort) {}

void GigaSerialBridge::begin(unsigned long baud, int receivePin,
                             int transmitPin) {
  serialPort_.begin(baud, SERIAL_8N1, receivePin, transmitPin);
}

bool GigaSerialBridge::sendPacket(const rivaler::Packet& packet) {
  uint8_t wireBytes[rivaler::kMaxPacketBytes]{};
  const uint8_t wireSize = rivaler::packetWireSize(packet);
  if (!rivaler::encodePacket(packet, wireBytes, sizeof(wireBytes))) {
    return false;
  }

  return serialPort_.write(wireBytes, wireSize) == wireSize;
}

bool GigaSerialBridge::readPacket(rivaler::Packet& packet) {
  constexpr uint8_t kMaximumBytesPerPoll = 64;
  uint8_t processedBytes = 0;

  while (serialPort_.available() > 0 && processedBytes < kMaximumBytesPerPoll) {
    ++processedBytes;
    if (parser_.pushByte(static_cast<uint8_t>(serialPort_.read()), packet)) {
      return true;
    }
  }

  return false;
}
