#pragma once

#include <Arduino.h>

#include "RivalerProtocol.h"

class MegaCommandReceiver {
 public:
  explicit MegaCommandReceiver(HardwareSerial& serialPort);

  void begin(unsigned long baud);
  bool readCommand(rivaler::CameraCommandPayload& command);

 private:
  bool isCommandValid(const rivaler::CameraCommandPayload& command) const;

  HardwareSerial& serialPort_;
  rivaler::PacketParser parser_;
};
