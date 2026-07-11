#include "LineSensors.h"

#include "Config.h"

using namespace rivaler_giga_config;

void LineSensors::begin() {
  analogReadResolution(kLineSensorAdcResolutionBits);
}

void LineSensors::update(unsigned long nowMs) {
  if (nowMs - lastSampleMs_ < kLineSensorSampleIntervalMs) {
    return;
  }

  lastSampleMs_ = nowMs;
  const uint16_t leftSample = analogRead(kLeftLineSensorPin);
  const uint16_t rightSample = analogRead(kRightLineSensorPin);

  if (!hasSample_) {
    leftFiltered_ = leftSample;
    rightFiltered_ = rightSample;
    hasSample_ = true;
    return;
  }

  leftFiltered_ =
      (leftFiltered_ * (kLineSensorFilterDivisor - 1) + leftSample) /
      kLineSensorFilterDivisor;
  rightFiltered_ =
      (rightFiltered_ * (kLineSensorFilterDivisor - 1) + rightSample) /
      kLineSensorFilterDivisor;
}

bool LineSensors::isLineDetected() const {
  return isLeftOnLine() || isRightOnLine();
}

bool LineSensors::isLeftOnLine() const {
  return hasSample_ && kLineDetectionEnabled && isLineValue(leftFiltered_);
}

bool LineSensors::isRightOnLine() const {
  return hasSample_ && kLineDetectionEnabled && isLineValue(rightFiltered_);
}

uint16_t LineSensors::leftReading() const { return leftFiltered_; }

uint16_t LineSensors::rightReading() const { return rightFiltered_; }

bool LineSensors::isLineValue(uint16_t value) {
  return kLineSensorHighMeansLine ? value >= kLineDetectThreshold
                                  : value <= kLineDetectThreshold;
}
