#include "DriveController.h"

#include "Config.h"

using namespace rivaler_giga_config;

namespace {

constexpr bool kFullStepPhases[4][2] = {
    {true, true},
    {false, true},
    {false, false},
    {true, false},
};

}  // namespace

void DriveController::begin() {
  analogWriteResolution(8);

  leftStepper_ = {kLeftStepperCoil1EnablePin,
                  kLeftStepperCoil1SpeedPin,
                  kLeftStepperCoil2EnablePin,
                  kLeftStepperCoil2SpeedPin,
                  0,
                  0,
                  0,
                  false,
                  kLeftDriveDirectionReversed};
  rightStepper_ = {kRightStepperCoil1EnablePin,
                   kRightStepperCoil1SpeedPin,
                   kRightStepperCoil2EnablePin,
                   kRightStepperCoil2SpeedPin,
                   0,
                   0,
                   0,
                   false,
                   kRightDriveDirectionReversed};

  const StepperState* steppers[] = {&leftStepper_, &rightStepper_};
  for (const StepperState* stepper : steppers) {
    pinMode(stepper->coil1DirectionPin, OUTPUT);
    pinMode(stepper->coil1PwmPin, OUTPUT);
    pinMode(stepper->coil2DirectionPin, OUTPUT);
    pinMode(stepper->coil2PwmPin, OUTPUT);
  }

  stop();
}

void DriveController::setJoystickCommand(int8_t joystickX, int8_t joystickY) {
  const int x = clampPercent(joystickX);
  const int y = clampPercent(joystickY);

  leftStepper_.commandPercent = clampPercent(y + x);
  rightStepper_.commandPercent = clampPercent(y - x);
}

void DriveController::update(unsigned long nowUs) {
  if (!kDriveOutputsEnabled) {
    releaseStepper(leftStepper_);
    releaseStepper(rightStepper_);
    return;
  }

  updateStepper(leftStepper_, nowUs);
  updateStepper(rightStepper_, nowUs);
}

void DriveController::stop() {
  leftStepper_.commandPercent = 0;
  rightStepper_.commandPercent = 0;
  releaseStepper(leftStepper_);
  releaseStepper(rightStepper_);
}

int DriveController::clampPercent(int value) {
  return constrain(value, -kMaximumDriveCommandPercent,
                   kMaximumDriveCommandPercent);
}

void DriveController::setCoil(int directionPin, int pwmPin, bool forward,
                              uint8_t pwmValue) {
  digitalWrite(directionPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, pwmValue);
}

void DriveController::releaseStepper(StepperState& stepper) {
  analogWrite(stepper.coil1PwmPin, 0);
  analogWrite(stepper.coil2PwmPin, 0);
  digitalWrite(stepper.coil1DirectionPin, LOW);
  digitalWrite(stepper.coil2DirectionPin, LOW);
  stepper.energized = false;
}

uint8_t DriveController::coilPwmForCommand(int commandPercent) {
  const int magnitude = abs(commandPercent);
  const long pwmValue = map(magnitude, 1, kMaximumDriveCommandPercent,
                            kStepperMinimumCoilPwm, kStepperMaximumCoilPwm);
  return static_cast<uint8_t>(
      constrain(pwmValue, static_cast<long>(kStepperMinimumCoilPwm),
                static_cast<long>(kStepperMaximumCoilPwm)));
}

unsigned long DriveController::stepIntervalUsForCommand(int commandPercent) {
  const int magnitude = abs(commandPercent);
  const long stepRateHz = map(magnitude, 1, kMaximumDriveCommandPercent,
                              kStepperMinimumRateHz, kStepperMaximumRateHz);
  return 1000000UL / static_cast<unsigned long>(stepRateHz);
}

void DriveController::applyPhase(StepperState& stepper, uint8_t pwmValue) {
  setCoil(stepper.coil1DirectionPin, stepper.coil1PwmPin,
          kFullStepPhases[stepper.phaseIndex][0], pwmValue);
  setCoil(stepper.coil2DirectionPin, stepper.coil2PwmPin,
          kFullStepPhases[stepper.phaseIndex][1], pwmValue);
  stepper.energized = true;
}

void DriveController::updateStepper(StepperState& stepper, unsigned long nowUs) {
  int command = stepper.commandPercent;
  if (stepper.directionReversed) {
    command = -command;
  }

  if (command == 0) {
    if (stepper.energized) {
      releaseStepper(stepper);
    }
    return;
  }

  const uint8_t pwmValue = coilPwmForCommand(command);
  if (!stepper.energized) {
    applyPhase(stepper, pwmValue);
    stepper.lastStepUs = nowUs;
    return;
  }

  if (nowUs - stepper.lastStepUs < stepIntervalUsForCommand(command)) {
    return;
  }

  stepper.phaseIndex =
      (stepper.phaseIndex + (command > 0 ? 1 : 3)) % 4;
  applyPhase(stepper, pwmValue);
  stepper.lastStepUs = nowUs;
}
