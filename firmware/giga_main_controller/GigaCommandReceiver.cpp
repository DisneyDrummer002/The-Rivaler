#include "GigaCommandReceiver.h"

GigaCommandReceiver::GigaCommandReceiver(HardwareSerial& serialPort)
    : serialPort_(serialPort) {}

void GigaCommandReceiver::begin(unsigned long baud) {
  serialPort_.begin(baud);
}

bool GigaCommandReceiver::readPacket(rivaler::Packet& packet) {
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

bool GigaCommandReceiver::sendPacket(const rivaler::Packet& packet) {
  uint8_t wireBytes[rivaler::kMaxPacketBytes]{};
  const uint8_t wireSize = rivaler::packetWireSize(packet);
  if (!rivaler::encodePacket(packet, wireBytes, sizeof(wireBytes))) {
    return false;
  }

  return serialPort_.write(wireBytes, wireSize) == wireSize;
}
