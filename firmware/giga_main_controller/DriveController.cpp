#include "DriveController.h"

#include "Config.h"

using namespace rivaler_giga_config;

namespace {

constexpr bool kFullStepPhases[4][2] = {
    {false, true},
    {false, false},
    {true, false},
    {true, true},
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
                  0,
                  kStepperStepIntervalUs,
                  false,
                  false,
                  kLeftDriveDirectionReversed};
  rightStepper_ = {kRightStepperCoil1EnablePin,
                   kRightStepperCoil1SpeedPin,
                   kRightStepperCoil2EnablePin,
                   kRightStepperCoil2SpeedPin,
                   0,
                   0,
                   0,
                   0,
                   kStepperStepIntervalUs,
                   true,
                   false,
                   kRightDriveDirectionReversed};

  const StepperState* steppers[] = {&leftStepper_, &rightStepper_};
  for (const StepperState* stepper : steppers) {
    pinMode(stepper->coil1DirectionPin, OUTPUT);
    pinMode(stepper->coil1EnablePin, OUTPUT);
    pinMode(stepper->coil2DirectionPin, OUTPUT);
    pinMode(stepper->coil2EnablePin, OUTPUT);
  }

  releaseStepper(leftStepper_);
  releaseStepper(rightStepper_);
}

void DriveController::setJoystickCommand(int8_t joystickX, int8_t joystickY) {
  const int x = clampPercent(joystickX);
  const int y = clampPercent(joystickY);
  const int xMagnitude = abs(x);
  const int yMagnitude = abs(y);

  if (xMagnitude <= kDriveJoystickDeadbandPercent &&
      yMagnitude <= kDriveJoystickDeadbandPercent) {
    setMotorCommands(0, 0);
    return;
  }

  // Restore continuous differential mixing. Diagonal joystick positions make
  // one wheel take fewer, more widely spaced steps instead of collapsing the
  // command to forward/backward/spin-only choices.
  setMotorCommands(y + x, y - x);
}

void DriveController::setSteeringCommand(int8_t steeringPercent,
                                         int8_t forwardPercent) {
  const int steering = clampPercent(steeringPercent);
  const int forward = clampPercent(forwardPercent);
  setMotorCommands(forward + steering, forward - steering);
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
  leftStepper_.queuedSteps = 0;
  rightStepper_.commandPercent = 0;
  rightStepper_.queuedSteps = 0;
}

int DriveController::clampPercent(int value) {
  return constrain(value, -kMaximumDriveCommandPercent,
                   kMaximumDriveCommandPercent);
}

void DriveController::setMotorCommands(int leftPercent, int rightPercent) {
  queueMotorCommand(leftStepper_, leftPercent);
  queueMotorCommand(rightStepper_, rightPercent);
}

void DriveController::queueMotorCommand(StepperState& stepper,
                                        int commandPercent) {
  const int command = clampPercent(commandPercent);
  if (command == 0) {
    stepper.commandPercent = 0;
    stepper.queuedSteps = 0;
    return;
  }

  const bool directionChanged =
      stepper.commandPercent != 0 &&
      ((stepper.commandPercent > 0) != (command > 0));
  if (directionChanged) {
    stepper.queuedSteps = 0;
  }

  stepper.commandPercent = command;
  const uint32_t commandMagnitude = static_cast<uint32_t>(abs(command));
  const uint16_t stepsForUpdate = static_cast<uint16_t>(max(
      1UL,
      (static_cast<uint32_t>(kStepperStepsPerCommandUpdate) *
           commandMagnitude +
       kMaximumDriveCommandPercent - 1) /
          kMaximumDriveCommandPercent));
  stepper.stepIntervalUs = max(
      kStepperStepIntervalUs,
      (kStepperStepIntervalUs *
       static_cast<unsigned long>(kMaximumDriveCommandPercent)) /
          commandMagnitude);
  const uint32_t expandedQueue =
      static_cast<uint32_t>(stepper.queuedSteps) +
      stepsForUpdate;
  stepper.queuedSteps = static_cast<uint16_t>(
      min(expandedQueue, static_cast<uint32_t>(kStepperMaximumQueuedSteps)));
}

void DriveController::enableStepper(StepperState& stepper) {
  if (stepper.usePwmEnable) {
    analogWrite(stepper.coil1EnablePin, kChannelsCdEnablePwm);
    analogWrite(stepper.coil2EnablePin, kChannelsCdEnablePwm);
  } else {
    digitalWrite(stepper.coil1EnablePin, HIGH);
    digitalWrite(stepper.coil2EnablePin, HIGH);
  }
  stepper.energized = true;
}

void DriveController::releaseStepper(StepperState& stepper) {
  if (stepper.usePwmEnable) {
    analogWrite(stepper.coil1EnablePin, 0);
    analogWrite(stepper.coil2EnablePin, 0);
  } else {
    digitalWrite(stepper.coil1EnablePin, LOW);
    digitalWrite(stepper.coil2EnablePin, LOW);
  }
  digitalWrite(stepper.coil1DirectionPin, LOW);
  digitalWrite(stepper.coil2DirectionPin, LOW);
  stepper.queuedSteps = 0;
  stepper.energized = false;
}

void DriveController::applyPhase(StepperState& stepper) {
  digitalWrite(stepper.coil1DirectionPin,
               kFullStepPhases[stepper.phaseIndex][0]);
  digitalWrite(stepper.coil2DirectionPin,
               kFullStepPhases[stepper.phaseIndex][1]);
}

void DriveController::updateStepper(StepperState& stepper, unsigned long nowUs) {
  int command = stepper.commandPercent;
  if (stepper.directionReversed) {
    command = -command;
  }

  if (command == 0 || stepper.queuedSteps == 0) {
    // Keep the last applied phase energized so the wheel remains locked and
    // motion can resume from the same point in the sequence.
    return;
  }

  if (!stepper.energized) {
    applyPhase(stepper);
    enableStepper(stepper);
    stepper.lastStepUs = nowUs;
    return;
  }

  if (nowUs - stepper.lastStepUs < stepper.stepIntervalUs) {
    return;
  }

  stepper.phaseIndex =
      (stepper.phaseIndex + (command > 0 ? 1 : 3)) % 4;
  applyPhase(stepper);
  --stepper.queuedSteps;
  // Resynchronize after every step instead of emitting rapid catch-up steps
  // after a slow sensor or communication pass.
  stepper.lastStepUs = nowUs;
}
