#pragma once

#include <Arduino.h>

class LineSensors {
 public:
  void begin();
  void update(unsigned long nowMs);

  bool isLineDetected() const;
  bool isLeftOnLine() const;
  bool isRightOnLine() const;
  uint16_t leftReading() const;
  uint16_t rightReading() const;

 private:
  static bool isLineValue(uint16_t value);

  uint16_t leftFiltered_ = 0;
  uint16_t rightFiltered_ = 0;
  bool hasSample_ = false;
  unsigned long lastSampleMs_ = 0;
};
