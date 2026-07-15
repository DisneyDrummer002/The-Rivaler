#pragma once

#include <Arduino.h>

class UltrasonicSafety {
 public:
  void begin();
  void update(unsigned long nowUs);

  uint16_t frontDistanceCm() const;
  uint16_t groundDistanceCm() const;
  bool isWallTooClose() const;
  bool isNoFloorDetected() const;
  bool hasSafetyWarning() const;
  bool isMeasurementInProgress() const;

 private:
  enum class MeasurementPhase : uint8_t {
    kIdle,
    kTriggerHigh,
    kWaitingForEchoStart,
    kWaitingForEchoEnd,
  };

  struct Sensor {
    int triggerPin;
    int echoPin;
    uint16_t filteredDistanceCm;
    bool hasMeasurement;
    bool lastMeasurementValid;
  };

  void completeMeasurement(bool valid, uint16_t distanceCm,
                           unsigned long nowUs);

  Sensor sensors_[2]{};
  MeasurementPhase phase_ = MeasurementPhase::kIdle;
  uint8_t activeSensorIndex_ = 0;
  unsigned long phaseStartedUs_ = 0;
  unsigned long echoStartedUs_ = 0;
  unsigned long lastMeasurementFinishedUs_ = 0;
};
