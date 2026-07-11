#pragma once

#include <stdint.h>

namespace rivaler_giga_config {

constexpr int kUnassignedPin = -1;

// UART-only exception to the GIGA's D20-and-higher external GPIO convention.
// Serial3 uses its dedicated pins; they are not general-purpose sensor pins.
constexpr unsigned long kRobotNanoUartBaud = 115200;
constexpr int kRobotNanoUartTxPin = 14;  // Serial3 TX, to Robot Nano D6.
constexpr int kRobotNanoUartRxPin = 15;  // Serial3 RX, from Robot Nano D7.
constexpr unsigned long kRemoteCommandTimeoutMs = 300;

// KS0448 L298P motor shield. These pins are fixed by the shield and must not
// be reused. Each bipolar stepper consumes two H-bridge channels.
constexpr int kMotorAEnablePin = 3;
constexpr int kMotorASpeedPin = 6;
constexpr int kMotorBEnablePin = 4;
constexpr int kMotorBSpeedPin = 5;
constexpr int kMotorCEnablePin = 7;
constexpr int kMotorCSpeedPin = 10;
constexpr int kMotorDEnablePin = 8;
constexpr int kMotorDSpeedPin = 9;

// Wire the left stepper to shield channels A and B, and the right stepper to
// channels C and D. Direction is corrected during the first wheel test.
constexpr int kLeftStepperCoil1EnablePin = kMotorAEnablePin;
constexpr int kLeftStepperCoil1SpeedPin = kMotorASpeedPin;
constexpr int kLeftStepperCoil2EnablePin = kMotorBEnablePin;
constexpr int kLeftStepperCoil2SpeedPin = kMotorBSpeedPin;
constexpr int kRightStepperCoil1EnablePin = kMotorCEnablePin;
constexpr int kRightStepperCoil1SpeedPin = kMotorCSpeedPin;
constexpr int kRightStepperCoil2EnablePin = kMotorDEnablePin;
constexpr int kRightStepperCoil2SpeedPin = kMotorDSpeedPin;

// The approved A/B-left and C/D-right channel plan is active. Motors remain
// stopped until a valid joystick command arrives from the remote.
constexpr bool kDriveOutputsEnabled = true;
constexpr int8_t kMaximumDriveCommandPercent = 60;
constexpr bool kLeftDriveDirectionReversed = false;
constexpr bool kRightDriveDirectionReversed = false;
constexpr uint8_t kStepperMinimumCoilPwm = 64;
constexpr uint8_t kStepperMaximumCoilPwm = 80;
constexpr uint16_t kStepperMinimumRateHz = 20;
constexpr uint16_t kStepperMaximumRateHz = 90;
constexpr unsigned long kRobotStatusTransmitIntervalMs = 250;

// Dedicated external H-bridge for the launcher DC motor. These three GIGA
// signals match common IN1/IN2/enable-PWM H-bridge modules.
constexpr bool kLauncherMotorDriverConfigured = true;
constexpr int kLauncherMotorInputAPin = 33;
constexpr int kLauncherMotorInputBPin = 34;
constexpr int kLauncherMotorEnablePwmPin = 35;
constexpr uint8_t kLauncherMotorPwm = 80;

// The magazine servo is continuous-rotation. Set motion enabled only after
// measuring the pulse values and duration that produce a 45-degree movement.
constexpr int kLauncherServoPin = 32;
constexpr bool kLauncherServoMotionEnabled = false;
constexpr uint16_t kLauncherServoNeutralPulseUs = 1500;
constexpr uint16_t kLauncherServoForwardPulseUs = 1600;
constexpr uint16_t kLauncherServoReversePulseUs = 1400;
constexpr unsigned long kLauncherServoQuarterTurnDurationMs = 0;

// Analog pins do not conflict with the shield's fixed motor-control pins.
constexpr int kLeftLineSensorPin = A0;
constexpr int kRightLineSensorPin = A1;
constexpr bool kLineDetectionEnabled = false;
constexpr uint16_t kLineDetectThreshold = 0;  // Calibrate before enabling.
constexpr bool kLineSensorHighMeansLine = true;
constexpr uint8_t kLineSensorAdcResolutionBits = 12;
constexpr unsigned long kLineSensorSampleIntervalMs = 10;
constexpr uint8_t kLineSensorFilterDivisor = 4;

// All external general-purpose digital connections are D22 or higher.
constexpr int kFrontUltrasonicTriggerPin = 22;
constexpr int kFrontUltrasonicEchoPin = 23;
constexpr int kDownUltrasonicTriggerPin = 24;
constexpr int kDownUltrasonicEchoPin = 25;
constexpr int kLeftUltrasonicTriggerPin = 26;
constexpr int kLeftUltrasonicEchoPin = 27;
constexpr int kRightUltrasonicTriggerPin = 28;
constexpr int kRightUltrasonicEchoPin = 29;
constexpr int kRearUltrasonicTriggerPin = 30;
constexpr int kRearUltrasonicEchoPin = 31;

// This project's HC-SR04 modules have been verified operating with the GIGA
// at 3.3 V, so their Echo signals connect directly to the configured inputs.
constexpr bool kUltrasonicEchoRequiresLevelShift = false;
constexpr bool kUltrasonicSafetyEnabled = false;
constexpr uint16_t kWallStopDistanceCm = 50;
constexpr uint16_t kNoFloorDistanceCm = 25;
constexpr unsigned long kUltrasonicTimeoutUs = 30000;
constexpr unsigned long kUltrasonicMeasurementIntervalUs = 60000;
constexpr unsigned long kUltrasonicTriggerPulseUs = 10;

constexpr unsigned long kLauncherArmDelayMs = 5000;
constexpr unsigned long kRapidFireShotIntervalMs = 3000;
constexpr uint8_t kRapidFireShotCount = 8;

}  // namespace rivaler_giga_config
