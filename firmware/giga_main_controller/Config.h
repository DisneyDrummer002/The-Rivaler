#pragma once

#include <stdint.h>

namespace rivaler_giga_config {

constexpr int kUnassignedPin = -1;

// UART-only exception to the GIGA's D20-and-higher external GPIO convention.
// Serial4 uses D14/D15 (TX3/RX3 on the board labels); they are not
// general-purpose sensor pins.
constexpr unsigned long kRobotNanoUartBaud = 115200;
constexpr int kRobotNanoUartTxPin = 14;  // Serial4 TX, to Robot Nano D6.
constexpr int kRobotNanoUartRxPin = 15;  // Serial4 RX, from Robot Nano D7.
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
constexpr int8_t kDriveJoystickDeadbandPercent = 12;
constexpr bool kStartupDriveTestEnabled = true;
constexpr unsigned long kStartupDriveTestDurationMs = 1000;
constexpr int8_t kStartupDriveTestPercent = 60;
constexpr bool kLeftDriveDirectionReversed = false;
constexpr bool kRightDriveDirectionReversed = false;
// Spread each joystick reading across its nominal 50 ms update window. The
// full-speed interval is currently 4 ms; lower commands receive proportionally
// fewer, more widely spaced steps for smooth differential steering.
constexpr unsigned long kStepperStepIntervalUs = 4000;
constexpr unsigned long kDriveCommandWindowUs = 50000;
constexpr uint16_t kStepperStepsPerCommandUpdate =
    kDriveCommandWindowUs / kStepperStepIntervalUs;
constexpr uint16_t kStepperMaximumQueuedSteps =
    kStepperStepsPerCommandUpdate * 2;
// Channels A/B remain ordinary digital enables. Only channels C/D use PWM.
constexpr uint8_t kChannelsCdEnablePwm = 255;
constexpr unsigned long kRobotStatusTransmitIntervalMs = 250;

// Dedicated external H-bridge for the launcher DC motor. The enable line is
// digital and drives the motor at full speed while armed.
constexpr bool kLauncherMotorDriverConfigured = true;
constexpr int kLauncherMotorInputAPin = 33;
constexpr int kLauncherMotorInputBPin = 34;
constexpr int kLauncherMotorEnablePin = 35;

// The magazine uses a 360-degree positional servo. Signed magazine angles are
// mapped directly to the servo's full pulse range; zero is the startup home.
constexpr int kLauncherServoPin = 11;
constexpr int16_t kLauncherServoMinimumAngleDeg = -180;
constexpr int16_t kLauncherServoMaximumAngleDeg = 180;
constexpr int16_t kLauncherServoHomeAngleDeg = 0;
constexpr uint16_t kLauncherServoMinimumPulseUs = 500;
constexpr uint16_t kLauncherServoMaximumPulseUs = 2500;
constexpr unsigned long kLauncherServoSettleDurationMs = 350;

// Analog pins do not conflict with the shield's fixed motor-control pins.
constexpr int kLeftLineSensorPin = A0;
constexpr int kRightLineSensorPin = A1;
constexpr uint8_t kLineSensorAdcResolutionBits = 12;
constexpr unsigned long kLineSensorSampleIntervalMs = 10;
constexpr uint8_t kLineSensorFilterDivisor = 4;

// Fill these four readings during bring-up. Each sensor needs an average raw
// reading over the normal floor and over the line. Either polarity is valid.
constexpr bool kLineSensorCalibrationConfigured = true;
constexpr uint16_t kLeftLineSensorFloorReading = 275;
constexpr uint16_t kLeftLineSensorLineReading = 4000;
constexpr uint16_t kRightLineSensorFloorReading = 275;
constexpr uint16_t kRightLineSensorLineReading = 4000;
constexpr uint16_t kLineSensorMinimumCalibrationSpan = 200;
constexpr uint16_t kLineDetectedStrength = 350;  // 0 to 1000.
constexpr unsigned long kLineSensorStaleTimeoutMs = 100;
constexpr bool kLineSensorCalibrationLoggingEnabled = true;
constexpr unsigned long kLineSensorCalibrationLogIntervalMs = 250;

// Line-following control. These are conservative first-pass values; tune them
// only after sensor calibration and a low-speed wheel test.
constexpr int8_t kLineFollowingCruisePercent = 36;
constexpr int8_t kLineFollowingMinimumForwardPercent = 20;
constexpr int8_t kLineFollowingMaximumCorrectionPercent = 45;
constexpr float kLineFollowingProportionalGain = 0.045f;
constexpr float kLineFollowingDerivativeGain = 0.00012f;
constexpr float kLineFollowingDerivativeFilter = 0.25f;
constexpr int8_t kLineFollowingCurveSlowdownPercent = 14;
constexpr int16_t kLineFollowingErrorDeadband = 20;
constexpr int16_t kLineFollowingSearchDirectionError = 80;
constexpr bool kLineFollowingSteeringReversed = false;
constexpr int8_t kLineFollowingManualOverridePercent = 15;
constexpr unsigned long kLineFollowingLossGraceMs = 180;
constexpr int8_t kLineFollowingGapForwardPercent = 15;
constexpr int8_t kLineFollowingSearchTurnPercent = 22;
constexpr unsigned long kLineFollowingSearchTimeoutMs = 1500;

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
constexpr bool kUltrasonicSafetyEnabled = true;
constexpr uint16_t kWallStopDistanceCm = 15;
constexpr uint16_t kNoFloorDistanceCm = 25;
constexpr bool kUltrasonicDiagnosticLoggingEnabled = false;
constexpr unsigned long kUltrasonicDiagnosticLogIntervalMs = 250;
constexpr bool kRemoteControlDiagnosticLoggingEnabled = false;
constexpr unsigned long kUltrasonicTimeoutUs = 30000;
constexpr unsigned long kUltrasonicMeasurementIntervalUs = 60000;
constexpr unsigned long kUltrasonicTriggerPulseUs = 10;

constexpr unsigned long kLauncherArmDelayMs = 5000;
constexpr unsigned long kRapidFireShotIntervalMs = 3000;
constexpr uint8_t kRapidFireShotCount = 8;

}  // namespace rivaler_giga_config
