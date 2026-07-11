#include "GigaSerialBridge.h"

GigaSerialBridge::GigaSerialBridge(HardwareSerial& serialPort)
    : serialPort_(serialPort) {}

void GigaSerialBridge::begin(unsigned long baud, int receivePin,
                             int transmitPin) {
  serialPort_.begin(baud, SERIAL_8N1, receivePin, transmitPin);
}

bool GigaSerialBridge::sendPacket(const rivaler::Packet& packet) {
  if (!rivaler::isPacketValid(packet, rivaler::packetWireSize(packet))) {
    return false;
  }

  const size_t wireSize = rivaler::packetWireSize(packet);
  return serialPort_.write(reinterpret_cast<const uint8_t*>(&packet),
                           wireSize) == wireSize;
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
