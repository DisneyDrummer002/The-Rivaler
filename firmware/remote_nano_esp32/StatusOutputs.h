#pragma once

#include <Arduino.h>

#include "RivalerProtocol.h"

class StatusOutputs {
 public:
  void begin();
  void applyRobotStatus(const rivaler::RobotStatusPayload& status);
  void setStatusLinkHealthy(bool isHealthy);
  void update(unsigned long nowMs);

 private:
  void clearStatusLights();

  bool safetyWarning_ = false;
  bool hasReceivedStatus_ = false;
  bool statusLinkHealthy_ = false;
  bool buzzerIsOn_ = false;
  unsigned long nextBuzzerChangeMs_ = 0;
};
