#pragma once

#include <Arduino.h>

class DriveController {
 public:
  void begin();
  void setJoystickCommand(int8_t joystickX, int8_t joystickY);
  void update(unsigned long nowUs);
  void stop();

 private:
  struct StepperState {
    int coil1DirectionPin;
    int coil1PwmPin;
    int coil2DirectionPin;
    int coil2PwmPin;
    int commandPercent;
    int phaseIndex;
    unsigned long lastStepUs;
    bool energized;
    bool directionReversed;
  };

  static int clampPercent(int value);
  static void setCoil(int directionPin, int pwmPin, bool forward,
                      uint8_t pwmValue);
  static void releaseStepper(StepperState& stepper);
  static uint8_t coilPwmForCommand(int commandPercent);
  static unsigned long stepIntervalUsForCommand(int commandPercent);
  static void applyPhase(StepperState& stepper, uint8_t pwmValue);
  static void updateStepper(StepperState& stepper, unsigned long nowUs);

  StepperState leftStepper_{};
  StepperState rightStepper_{};
};
