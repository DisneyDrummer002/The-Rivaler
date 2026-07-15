#pragma once

#include <Arduino.h>

class LineSensors {
 public:
  void begin();
  void update(unsigned long nowMs);

  bool isLineDetected() const;
  bool isLeftOnLine() const;
  bool isRightOnLine() const;
  bool isCalibrationValid() const;
  bool hasFreshSample(unsigned long nowMs) const;
  uint16_t leftReading() const;
  uint16_t rightReading() const;
  uint16_t leftLineStrength() const;
  uint16_t rightLineStrength() const;
  unsigned long sampleTimestampMs() const;

 private:
  static uint16_t normalizedLineStrength(uint16_t reading,
                                         uint16_t floorReading,
                                         uint16_t lineReading);

  uint16_t leftFiltered_ = 0;
  uint16_t rightFiltered_ = 0;
  bool hasSample_ = false;
  unsigned long lastSampleMs_ = 0;
};
