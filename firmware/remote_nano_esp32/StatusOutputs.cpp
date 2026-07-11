#include "StatusOutputs.h"

#include "Config.h"

using namespace rivaler_remote_config;

void StatusOutputs::begin() {
  pinMode(kBuzzerPin, OUTPUT);
  pinMode(kLauncherArmedLedPin, OUTPUT);
  pinMode(kLineDetectedLedPin, OUTPUT);
  pinMode(kLineFollowingLedPin, OUTPUT);

  digitalWrite(kBuzzerPin, LOW);
  digitalWrite(kLauncherArmedLedPin, LOW);
  digitalWrite(kLineDetectedLedPin, LOW);
  digitalWrite(kLineFollowingLedPin, LOW);
}

void StatusOutputs::applyRobotStatus(
    const rivaler::RobotStatusPayload& status) {
  hasReceivedStatus_ = true;
  digitalWrite(kLauncherArmedLedPin,
               (status.statusFlags & rivaler::kLauncherArmed) ? HIGH : LOW);
  digitalWrite(kLineDetectedLedPin,
               (status.statusFlags & rivaler::kLineDetected) ? HIGH : LOW);
  digitalWrite(kLineFollowingLedPin,
               (status.statusFlags & rivaler::kLineFollowingActive) ? HIGH
                                                                     : LOW);
  safetyWarning_ = (status.statusFlags & rivaler::kSafetyWarning) != 0;
}

void StatusOutputs::setStatusLinkHealthy(bool isHealthy) {
  if (!isHealthy && hasReceivedStatus_ && statusLinkHealthy_) {
    clearStatusLights();
    safetyWarning_ = false;
  }

  statusLinkHealthy_ = isHealthy;
}

void StatusOutputs::update(unsigned long nowMs) {
  const bool linkLost = hasReceivedStatus_ && !statusLinkHealthy_;
  const bool shouldBuzz = safetyWarning_ || linkLost;
  if (!shouldBuzz) {
    if (buzzerIsOn_) {
      buzzerIsOn_ = false;
      digitalWrite(kBuzzerPin, LOW);
    }
    return;
  }

  if (nowMs < nextBuzzerChangeMs_) {
    return;
  }

  buzzerIsOn_ = !buzzerIsOn_;
  digitalWrite(kBuzzerPin, buzzerIsOn_ ? HIGH : LOW);
  if (safetyWarning_) {
    nextBuzzerChangeMs_ =
        nowMs + (buzzerIsOn_ ? kSafetyBuzzerOnMs : kSafetyBuzzerOffMs);
  } else {
    nextBuzzerChangeMs_ =
        nowMs + (buzzerIsOn_ ? kLinkLostBuzzerOnMs : kLinkLostBuzzerOffMs);
  }
}

void StatusOutputs::clearStatusLights() {
  digitalWrite(kLauncherArmedLedPin, LOW);
  digitalWrite(kLineDetectedLedPin, LOW);
  digitalWrite(kLineFollowingLedPin, LOW);
}
