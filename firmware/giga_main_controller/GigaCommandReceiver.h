#pragma once

#include <Arduino.h>

#include "RivalerProtocol.h"

class GigaCommandReceiver {
 public:
  explicit GigaCommandReceiver(HardwareSerial& serialPort);

  void begin(unsigned long baud);
  bool readPacket(rivaler::Packet& packet);
  bool sendPacket(const rivaler::Packet& packet);

 private:
  HardwareSerial& serialPort_;
  rivaler::PacketParser parser_;
};
