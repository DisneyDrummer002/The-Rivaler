#include "LineSensors.h"

#include "Config.h"

using namespace rivaler_giga_config;

static_assert(kLineSensorFilterDivisor > 0,
              "Line sensor filter divisor must be nonzero");
static_assert(kLineDetectedStrength <= 1000,
              "Line detection strength must be between 0 and 1000");

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
  return hasSample_ && isCalibrationValid() &&
         leftLineStrength() >= kLineDetectedStrength;
}

bool LineSensors::isRightOnLine() const {
  return hasSample_ && isCalibrationValid() &&
         rightLineStrength() >= kLineDetectedStrength;
}

bool LineSensors::isCalibrationValid() const {
  const int32_t leftSpan =
      static_cast<int32_t>(kLeftLineSensorLineReading) -
      static_cast<int32_t>(kLeftLineSensorFloorReading);
  const int32_t rightSpan =
      static_cast<int32_t>(kRightLineSensorLineReading) -
      static_cast<int32_t>(kRightLineSensorFloorReading);
  return kLineSensorCalibrationConfigured &&
         abs(leftSpan) >= kLineSensorMinimumCalibrationSpan &&
         abs(rightSpan) >= kLineSensorMinimumCalibrationSpan;
}

bool LineSensors::hasFreshSample(unsigned long nowMs) const {
  return hasSample_ &&
         nowMs - lastSampleMs_ <= kLineSensorStaleTimeoutMs;
}

uint16_t LineSensors::leftReading() const { return leftFiltered_; }

uint16_t LineSensors::rightReading() const { return rightFiltered_; }

uint16_t LineSensors::leftLineStrength() const {
  return normalizedLineStrength(leftFiltered_, kLeftLineSensorFloorReading,
                                kLeftLineSensorLineReading);
}

uint16_t LineSensors::rightLineStrength() const {
  return normalizedLineStrength(rightFiltered_, kRightLineSensorFloorReading,
                                kRightLineSensorLineReading);
}

unsigned long LineSensors::sampleTimestampMs() const { return lastSampleMs_; }

uint16_t LineSensors::normalizedLineStrength(uint16_t reading,
                                             uint16_t floorReading,
                                             uint16_t lineReading) {
  const int32_t span = static_cast<int32_t>(lineReading) - floorReading;
  if (abs(span) < kLineSensorMinimumCalibrationSpan) {
    return 0;
  }

  const int32_t offset = static_cast<int32_t>(reading) - floorReading;
  const int32_t strength = (offset * 1000L) / span;
  return static_cast<uint16_t>(constrain(strength, 0L, 1000L));
}
