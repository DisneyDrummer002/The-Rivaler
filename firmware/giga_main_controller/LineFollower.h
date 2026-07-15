#pragma once

#include <Arduino.h>

class DriveController;
class LineSensors;

class LineFollower {
 public:
  bool activate(const LineSensors& sensors, unsigned long nowMs);
  void deactivate();
  void update(const LineSensors& sensors, DriveController& drive,
              unsigned long nowMs);

  bool isActive() const;
  bool isRecovering() const;

 private:
  enum class State : uint8_t {
    kInactive,
    kTracking,
    kRecovering,
  };

  void applyTrackingCommand(const LineSensors& sensors,
                            DriveController& drive,
                            unsigned long sampleTimestampMs,
                            unsigned long nowMs);
  void applyRecoveryCommand(DriveController& drive, unsigned long nowMs);

  State state_ = State::kInactive;
  bool hasProcessedSample_ = false;
  bool hasPreviousError_ = false;
  unsigned long lastProcessedSampleMs_ = 0;
  unsigned long lastLineSeenMs_ = 0;
  int16_t previousError_ = 0;
  float filteredDerivative_ = 0.0f;
  int8_t lastCorrectionPercent_ = 0;
  int8_t searchDirection_ = 0;
};
