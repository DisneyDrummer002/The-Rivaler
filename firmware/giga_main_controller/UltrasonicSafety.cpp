#include "UltrasonicSafety.h"

#include "Config.h"

using namespace rivaler_giga_config;

namespace {

constexpr uint8_t kFrontSensorIndex = 0;
constexpr uint8_t kDownSensorIndex = 1;
constexpr uint8_t kFilterDivisor = 4;

}  // namespace

void UltrasonicSafety::begin() {
  sensors_[kFrontSensorIndex] = {kFrontUltrasonicTriggerPin,
                                 kFrontUltrasonicEchoPin, 0, false, false};
  sensors_[kDownSensorIndex] = {kDownUltrasonicTriggerPin,
                                kDownUltrasonicEchoPin, 0, false, false};

  for (Sensor& sensor : sensors_) {
    pinMode(sensor.triggerPin, OUTPUT);
    pinMode(sensor.echoPin, INPUT);
    digitalWrite(sensor.triggerPin, LOW);
  }
}

void UltrasonicSafety::update(unsigned long nowUs) {
  if (!kUltrasonicSafetyEnabled) {
    return;
  }

  Sensor& activeSensor = sensors_[activeSensorIndex_];
  switch (phase_) {
    case MeasurementPhase::kIdle:
      if (nowUs - lastMeasurementFinishedUs_ <
          kUltrasonicMeasurementIntervalUs) {
        return;
      }
      digitalWrite(activeSensor.triggerPin, HIGH);
      phaseStartedUs_ = nowUs;
      phase_ = MeasurementPhase::kTriggerHigh;
      return;

    case MeasurementPhase::kTriggerHigh:
      if (nowUs - phaseStartedUs_ < kUltrasonicTriggerPulseUs) {
        return;
      }
      digitalWrite(activeSensor.triggerPin, LOW);
      phaseStartedUs_ = nowUs;
      phase_ = MeasurementPhase::kWaitingForEchoStart;
      return;

    case MeasurementPhase::kWaitingForEchoStart:
      if (digitalRead(activeSensor.echoPin) == HIGH) {
        echoStartedUs_ = nowUs;
        phase_ = MeasurementPhase::kWaitingForEchoEnd;
        return;
      }
      if (nowUs - phaseStartedUs_ >= kUltrasonicTimeoutUs) {
        completeMeasurement(false, 0, nowUs);
      }
      return;

    case MeasurementPhase::kWaitingForEchoEnd:
      if (digitalRead(activeSensor.echoPin) == LOW) {
        const unsigned long echoDurationUs = nowUs - echoStartedUs_;
        const uint16_t distanceCm = static_cast<uint16_t>(echoDurationUs / 58);
        completeMeasurement(distanceCm > 0, distanceCm, nowUs);
        return;
      }
      if (nowUs - echoStartedUs_ >= kUltrasonicTimeoutUs) {
        completeMeasurement(false, 0, nowUs);
      }
      return;
  }
}

uint16_t UltrasonicSafety::frontDistanceCm() const {
  const Sensor& sensor = sensors_[kFrontSensorIndex];
  return sensor.lastMeasurementValid ? sensor.filteredDistanceCm : 0;
}

uint16_t UltrasonicSafety::groundDistanceCm() const {
  const Sensor& sensor = sensors_[kDownSensorIndex];
  return sensor.lastMeasurementValid ? sensor.filteredDistanceCm : 0;
}

bool UltrasonicSafety::isWallTooClose() const {
  const Sensor& frontSensor = sensors_[kFrontSensorIndex];
  return kUltrasonicSafetyEnabled && frontSensor.lastMeasurementValid &&
         frontSensor.filteredDistanceCm <= kWallStopDistanceCm;
}

bool UltrasonicSafety::isNoFloorDetected() const {
  const Sensor& downSensor = sensors_[kDownSensorIndex];
  if (!kUltrasonicSafetyEnabled || !downSensor.hasMeasurement) {
    return false;
  }

  return !downSensor.lastMeasurementValid ||
         downSensor.filteredDistanceCm > kNoFloorDistanceCm;
}

bool UltrasonicSafety::hasSafetyWarning() const {
  return isWallTooClose() || isNoFloorDetected();
}

void UltrasonicSafety::completeMeasurement(bool valid, uint16_t distanceCm,
                                           unsigned long nowUs) {
  Sensor& activeSensor = sensors_[activeSensorIndex_];
  activeSensor.hasMeasurement = true;
  activeSensor.lastMeasurementValid = valid;

  if (valid) {
    if (activeSensor.filteredDistanceCm == 0) {
      activeSensor.filteredDistanceCm = distanceCm;
    } else {
      activeSensor.filteredDistanceCm =
          (activeSensor.filteredDistanceCm * (kFilterDivisor - 1) +
           distanceCm) /
          kFilterDivisor;
    }
  }

  activeSensorIndex_ = (activeSensorIndex_ + 1) % 2;
  phase_ = MeasurementPhase::kIdle;
  lastMeasurementFinishedUs_ = nowUs;
}
