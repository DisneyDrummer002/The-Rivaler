#pragma once

#include <Arduino.h>

#include "RivalerProtocol.h"

class MegaCameraCommand {
 public:
  explicit MegaCameraCommand(HardwareSerial& serialPort);

  void begin(unsigned long baud, int transmitPin);
  void sendForRemoteControl(const rivaler::RemoteControlPayload& control,
                            uint8_t& nextSequence);

 private:
  void sendCaptureBurst(uint8_t photoCount, uint16_t intervalMs,
                        uint8_t& nextSequence);
  void sendPrepareSdEject(uint8_t& nextSequence);
  bool sendPacket(const rivaler::Packet& packet);

  HardwareSerial& serialPort_;
};
