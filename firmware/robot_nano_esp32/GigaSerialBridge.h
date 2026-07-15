#pragma once

#include <Arduino.h>

#include "RivalerProtocol.h"

class GigaSerialBridge {
 public:
  explicit GigaSerialBridge(HardwareSerial& serialPort);

  void begin(unsigned long baud, int receivePin, int transmitPin);
  bool sendPacket(const rivaler::Packet& packet);
  bool readPacket(rivaler::Packet& packet);

 private:
  HardwareSerial& serialPort_;
  rivaler::PacketParser parser_;
};
