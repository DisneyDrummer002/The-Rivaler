#include "LauncherController.h"

#include "Config.h"

using namespace rivaler_giga_config;

namespace {

constexpr int16_t kShotPositionsDegrees[kRapidFireShotCount] = {
    90, 180, 270, 360, -90, -180, -270, 0,
};

static_assert(kRapidFireShotCount ==
                  sizeof(kShotPositionsDegrees) /
                      sizeof(kShotPositionsDegrees[0]),
              "Rapid-fire position count must match the shot count");
static_assert(kLauncherServoMinimumAngleDeg < kLauncherServoHomeAngleDeg &&
                  kLauncherServoHomeAngleDeg < kLauncherServoMaximumAngleDeg,
              "Launcher home angle must be inside the servo range");

}  // namespace

void LauncherController::begin() {
  if (kLauncherMotorDriverConfigured) {
    pinMode(kLauncherMotorInputAPin, OUTPUT);
    pinMode(kLauncherMotorInputBPin, OUTPUT);
    pinMode(kLauncherMotorEnablePin, OUTPUT);
  }

  magazineServo_.attach(kLauncherServoPin, kLauncherServoMinimumPulseUs,
                        kLauncherServoMaximumPulseUs);
  writeMagazineAngle(kLauncherServoHomeAngleDeg);
  disarm();
}

void LauncherController::update(bool armSwitchHeld, bool safetyStopActive,
                                unsigned long nowMs) {
  if (!kLauncherMotorDriverConfigured || safetyStopActive || !armSwitchHeld) {
    disarm();
    return;
  }

  if (state_ == State::kDisarmed) {
    startMotor();
    armingStartedMs_ = nowMs;
    state_ = State::kArming;
    return;
  }

  if (state_ == State::kArming &&
      nowMs - armingStartedMs_ >= kLauncherArmDelayMs) {
    state_ = State::kArmed;
  }

  updateMagazine(nowMs);
}

bool LauncherController::requestQuickShoot(unsigned long nowMs) {
  if (!isArmed() || magazineState_ != MagazineState::kAtRest ||
      rapidFireActive_) {
    return false;
  }

  moveToNextShotPosition(nowMs);
  return true;
}

bool LauncherController::requestRapidFire(unsigned long nowMs) {
  if (!isArmed() || magazineState_ != MagazineState::kAtRest ||
      rapidFireActive_) {
    return false;
  }

  rapidFireActive_ = true;
  rapidShotsCompleted_ = 0;
  moveToNextShotPosition(nowMs);
  return true;
}

bool LauncherController::isArmed() const { return state_ == State::kArmed; }

bool LauncherController::isArming() const {
  return state_ == State::kArming;
}

bool LauncherController::isQuickShootActive() const {
  return magazineState_ != MagazineState::kAtRest;
}

void LauncherController::disarm() {
  stopMotor();
  stopMagazine();
  state_ = State::kDisarmed;
  armingStartedMs_ = 0;
}

void LauncherController::startMotor() {
  if (!kLauncherMotorDriverConfigured) {
    return;
  }

  digitalWrite(kLauncherMotorInputAPin, HIGH);
  digitalWrite(kLauncherMotorInputBPin, LOW);
  digitalWrite(kLauncherMotorEnablePin, HIGH);
}

void LauncherController::stopMotor() {
  if (!kLauncherMotorDriverConfigured) {
    return;
  }

  digitalWrite(kLauncherMotorEnablePin, LOW);
  digitalWrite(kLauncherMotorInputAPin, LOW);
  digitalWrite(kLauncherMotorInputBPin, LOW);
}

void LauncherController::moveToNextShotPosition(unsigned long nowMs) {
  writeMagazineAngle(kShotPositionsDegrees[nextShotPositionIndex_]);
  nextShotPositionIndex_ =
      (nextShotPositionIndex_ + 1) % kRapidFireShotCount;
  magazineMotionStartedMs_ = nowMs;
  magazineState_ = MagazineState::kMovingToShotPosition;
}

void LauncherController::writeMagazineAngle(int16_t angleDegrees) {
  const int16_t constrainedAngle = constrain(
      angleDegrees, kLauncherServoMinimumAngleDeg,
      kLauncherServoMaximumAngleDeg);
  const long pulseWidthUs = map(
      constrainedAngle, kLauncherServoMinimumAngleDeg,
      kLauncherServoMaximumAngleDeg, kLauncherServoMinimumPulseUs,
      kLauncherServoMaximumPulseUs);
  magazineServo_.writeMicroseconds(static_cast<int>(pulseWidthUs));
  magazineAngleDegrees_ = constrainedAngle;
}

void LauncherController::stopMagazine() {
  magazineState_ = MagazineState::kAtRest;
  rapidFireActive_ = false;
  rapidShotsCompleted_ = 0;
  magazineMotionStartedMs_ = 0;
}

void LauncherController::updateMagazine(unsigned long nowMs) {
  if (magazineState_ == MagazineState::kAtRest) {
    return;
  }

  if (magazineState_ == MagazineState::kWaitingBetweenRapidShots) {
    if (nowMs - magazineMotionStartedMs_ < kRapidFireShotIntervalMs) {
      return;
    }

    moveToNextShotPosition(nowMs);
    return;
  }

  if (nowMs - magazineMotionStartedMs_ < kLauncherServoSettleDurationMs) {
    return;
  }

  if (rapidFireActive_) {
    ++rapidShotsCompleted_;
    if (rapidShotsCompleted_ >= kRapidFireShotCount) {
      stopMagazine();
      return;
    }

    magazineMotionStartedMs_ = nowMs;
    magazineState_ = MagazineState::kWaitingBetweenRapidShots;
    return;
  }

  stopMagazine();
}
