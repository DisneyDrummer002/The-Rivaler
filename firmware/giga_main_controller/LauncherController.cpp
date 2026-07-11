#include "LauncherController.h"

#include "Config.h"

using namespace rivaler_giga_config;

void LauncherController::begin() {
  if (kLauncherMotorDriverConfigured) {
    pinMode(kLauncherMotorInputAPin, OUTPUT);
    pinMode(kLauncherMotorInputBPin, OUTPUT);
    pinMode(kLauncherMotorEnablePwmPin, OUTPUT);
  }

  magazineServo_.attach(kLauncherServoPin);
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
  if (!isArmed() || !kLauncherServoMotionEnabled ||
      kLauncherServoQuarterTurnDurationMs == 0 ||
      magazineState_ != MagazineState::kAtRest || rapidFireActive_) {
    return false;
  }

  magazineServo_.writeMicroseconds(kLauncherServoForwardPulseUs);
  magazineMotionStartedMs_ = nowMs;
  magazineState_ = MagazineState::kMovingForward;
  return true;
}

bool LauncherController::requestRapidFire(unsigned long nowMs) {
  if (!isArmed() || !kLauncherServoMotionEnabled ||
      kLauncherServoQuarterTurnDurationMs == 0 ||
      magazineState_ != MagazineState::kAtRest || rapidFireActive_) {
    return false;
  }

  rapidFireActive_ = true;
  rapidShotsCompleted_ = 0;
  magazineServo_.writeMicroseconds(kLauncherServoForwardPulseUs);
  magazineMotionStartedMs_ = nowMs;
  magazineState_ = MagazineState::kMovingForward;
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
  analogWrite(kLauncherMotorEnablePwmPin, kLauncherMotorPwm);
}

void LauncherController::stopMotor() {
  if (!kLauncherMotorDriverConfigured) {
    return;
  }

  analogWrite(kLauncherMotorEnablePwmPin, 0);
  digitalWrite(kLauncherMotorInputAPin, LOW);
  digitalWrite(kLauncherMotorInputBPin, LOW);
}

void LauncherController::stopMagazine() {
  magazineServo_.writeMicroseconds(kLauncherServoNeutralPulseUs);
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

    magazineServo_.writeMicroseconds(kLauncherServoForwardPulseUs);
    magazineMotionStartedMs_ = nowMs;
    magazineState_ = MagazineState::kMovingForward;
    return;
  }

  if (nowMs - magazineMotionStartedMs_ <
      kLauncherServoQuarterTurnDurationMs) {
    return;
  }

  if (magazineState_ == MagazineState::kMovingForward) {
    magazineServo_.writeMicroseconds(kLauncherServoReversePulseUs);
    magazineMotionStartedMs_ = nowMs;
    magazineState_ = MagazineState::kReturning;
    return;
  }

  if (rapidFireActive_) {
    ++rapidShotsCompleted_;
    if (rapidShotsCompleted_ >= kRapidFireShotCount) {
      stopMagazine();
      return;
    }

    magazineServo_.writeMicroseconds(kLauncherServoNeutralPulseUs);
    magazineMotionStartedMs_ = nowMs;
    magazineState_ = MagazineState::kWaitingBetweenRapidShots;
    return;
  }

  stopMagazine();
}
