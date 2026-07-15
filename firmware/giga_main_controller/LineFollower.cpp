#include "LineFollower.h"

#include "Config.h"
#include "DriveController.h"
#include "LineSensors.h"

using namespace rivaler_giga_config;

static_assert(kLineFollowingMinimumForwardPercent > 0,
              "Line following must use a positive minimum speed");
static_assert(kLineFollowingMinimumForwardPercent <=
                  kLineFollowingCruisePercent,
              "Line following minimum speed exceeds cruise speed");
static_assert(kLineFollowingCruisePercent <= kMaximumDriveCommandPercent,
              "Line following cruise speed exceeds the drive limit");
static_assert(kLineFollowingMaximumCorrectionPercent <=
                  kMaximumDriveCommandPercent,
              "Line following correction exceeds the drive limit");
static_assert(kLineFollowingDerivativeFilter > 0.0f &&
                  kLineFollowingDerivativeFilter <= 1.0f,
              "Derivative filter must be in the range (0, 1]");
static_assert(kLineFollowingLossGraceMs < kLineFollowingSearchTimeoutMs,
              "Line recovery grace must be shorter than its timeout");

bool LineFollower::activate(const LineSensors& sensors,
                            unsigned long nowMs) {
  if (!sensors.isCalibrationValid() || !sensors.hasFreshSample(nowMs) ||
      !sensors.isLineDetected()) {
    return false;
  }

  deactivate();
  state_ = State::kTracking;
  lastLineSeenMs_ = nowMs;
  return true;
}

void LineFollower::deactivate() {
  state_ = State::kInactive;
  hasProcessedSample_ = false;
  hasPreviousError_ = false;
  lastProcessedSampleMs_ = 0;
  lastLineSeenMs_ = 0;
  previousError_ = 0;
  filteredDerivative_ = 0.0f;
  lastCorrectionPercent_ = 0;
  searchDirection_ = 0;
}

void LineFollower::update(const LineSensors& sensors, DriveController& drive,
                          unsigned long nowMs) {
  if (!isActive()) {
    return;
  }

  if (!sensors.isCalibrationValid() || !sensors.hasFreshSample(nowMs)) {
    deactivate();
    drive.stop();
    return;
  }

  const unsigned long sampleTimestampMs = sensors.sampleTimestampMs();
  if (hasProcessedSample_ &&
      sampleTimestampMs == lastProcessedSampleMs_) {
    return;
  }

  hasProcessedSample_ = true;
  if (sensors.isLineDetected()) {
    applyTrackingCommand(sensors, drive, sampleTimestampMs, nowMs);
  } else {
    applyRecoveryCommand(drive, nowMs);
  }
  lastProcessedSampleMs_ = sampleTimestampMs;
}

bool LineFollower::isActive() const {
  return state_ != State::kInactive;
}

bool LineFollower::isRecovering() const {
  return state_ == State::kRecovering;
}

void LineFollower::applyTrackingCommand(const LineSensors& sensors,
                                        DriveController& drive,
                                        unsigned long sampleTimestampMs,
                                        unsigned long nowMs) {
  int16_t error = static_cast<int16_t>(sensors.rightLineStrength()) -
                  static_cast<int16_t>(sensors.leftLineStrength());
  if (kLineFollowingSteeringReversed) {
    error = -error;
  }
  if (abs(error) <= kLineFollowingErrorDeadband) {
    error = 0;
  }

  if (state_ == State::kRecovering) {
    hasPreviousError_ = false;
    filteredDerivative_ = 0.0f;
  }

  float rawDerivative = 0.0f;
  if (hasPreviousError_) {
    const unsigned long elapsedMs =
        sampleTimestampMs - lastProcessedSampleMs_;
    if (elapsedMs > 0) {
      rawDerivative = static_cast<float>(error - previousError_) *
                      1000.0f / static_cast<float>(elapsedMs);
    }
  }
  filteredDerivative_ += kLineFollowingDerivativeFilter *
                         (rawDerivative - filteredDerivative_);

  const float correction =
      kLineFollowingProportionalGain * static_cast<float>(error) +
      kLineFollowingDerivativeGain * filteredDerivative_;
  const int roundedCorrection = static_cast<int>(
      correction + (correction >= 0.0f ? 0.5f : -0.5f));
  lastCorrectionPercent_ = static_cast<int8_t>(constrain(
      roundedCorrection, -kLineFollowingMaximumCorrectionPercent,
      kLineFollowingMaximumCorrectionPercent));

  const int slowdown =
      (abs(error) * kLineFollowingCurveSlowdownPercent) / 1000;
  const int forwardPercent = constrain(
      kLineFollowingCruisePercent - slowdown,
      kLineFollowingMinimumForwardPercent, kLineFollowingCruisePercent);
  drive.setSteeringCommand(lastCorrectionPercent_,
                           static_cast<int8_t>(forwardPercent));

  if (abs(error) >= kLineFollowingSearchDirectionError) {
    searchDirection_ = error > 0 ? 1 : -1;
  }
  previousError_ = error;
  hasPreviousError_ = true;
  lastLineSeenMs_ = nowMs;
  state_ = State::kTracking;
}

void LineFollower::applyRecoveryCommand(DriveController& drive,
                                        unsigned long nowMs) {
  state_ = State::kRecovering;
  const unsigned long lineLostForMs = nowMs - lastLineSeenMs_;
  if (lineLostForMs <= kLineFollowingLossGraceMs) {
    const int correction = constrain(
        static_cast<int>(lastCorrectionPercent_),
        -static_cast<int>(kLineFollowingGapForwardPercent),
        static_cast<int>(kLineFollowingGapForwardPercent));
    drive.setSteeringCommand(static_cast<int8_t>(correction),
                             kLineFollowingGapForwardPercent);
    return;
  }

  if (searchDirection_ != 0 &&
      lineLostForMs <= kLineFollowingSearchTimeoutMs) {
    drive.setSteeringCommand(
        searchDirection_ * kLineFollowingSearchTurnPercent, 0);
    return;
  }

  deactivate();
  drive.stop();
}
