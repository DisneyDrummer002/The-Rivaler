#pragma once

#include <Arduino.h>

class DriveController {
 public:
  void begin();
  void setJoystickCommand(int8_t joystickX, int8_t joystickY);
  void setSteeringCommand(int8_t steeringPercent, int8_t forwardPercent);
  void update(unsigned long nowUs);
  void stop();

 private:
  struct StepperState {
    int coil1DirectionPin;
    int coil1EnablePin;
    int coil2DirectionPin;
    int coil2EnablePin;
    int commandPercent;
    int phaseIndex;
    uint16_t queuedSteps;
    unsigned long lastStepUs;
    unsigned long stepIntervalUs;
    bool usePwmEnable;
    bool energized;
    bool directionReversed;
  };

  static int clampPercent(int value);
  void setMotorCommands(int leftPercent, int rightPercent);
  static void queueMotorCommand(StepperState& stepper, int commandPercent);
  static void enableStepper(StepperState& stepper);
  static void releaseStepper(StepperState& stepper);
  static void applyPhase(StepperState& stepper);
  static void updateStepper(StepperState& stepper, unsigned long nowUs);

  StepperState leftStepper_{};
  StepperState rightStepper_{};
};
