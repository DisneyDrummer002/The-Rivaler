#include "Config.h"
#include "DriveController.h"
#include "GigaCommandReceiver.h"
#include "LauncherController.h"
#include "LineFollower.h"
#include "LineSensors.h"
#include "UltrasonicSafety.h"

namespace {

// D14/D15 are the GIGA's Serial4 pins (TX3/RX3 on the board labels).
GigaCommandReceiver robotLink(Serial4);
DriveController driveController;
LauncherController launcherController;
LineFollower lineFollower;
LineSensors lineSensors;
UltrasonicSafety ultrasonicSafety;

bool hasReceivedControl = false;
bool emergencyStopActive = false;
bool remoteTimeoutActive = false;
bool launcherArmRequested = false;
unsigned long lastControlReceivedMs = 0;
unsigned long lastStatusTransmitMs = 0;
unsigned long lastLineCalibrationPrintMs = 0;
unsigned long lastUltrasonicDiagnosticPrintMs = 0;
uint8_t nextStatusSequence = 0;
unsigned long startupDriveTestStartedMs = 0;
bool startupDriveTestComplete = false;
bool remoteControlDiagnosticPending = false;
uint8_t remoteControlDiagnosticSequence = 0;
rivaler::RemoteControlPayload remoteControlDiagnostic{};

rivaler::SafetyFault activeSafetyFault() {
  if (remoteTimeoutActive) {
    return rivaler::SafetyFault::kRemoteTimeout;
  }
  if (emergencyStopActive) {
    return rivaler::SafetyFault::kEmergencyStop;
  }
  if (ultrasonicSafety.isNoFloorDetected()) {
    return rivaler::SafetyFault::kNoFloorDetected;
  }
  if (ultrasonicSafety.isWallTooClose()) {
    return rivaler::SafetyFault::kWallTooClose;
  }
  return rivaler::SafetyFault::kNone;
}

const char* safetyFaultName(rivaler::SafetyFault fault) {
  switch (fault) {
    case rivaler::SafetyFault::kNone:
      return "none";
    case rivaler::SafetyFault::kRemoteTimeout:
      return "remote-timeout";
    case rivaler::SafetyFault::kEmergencyStop:
      return "emergency-stop";
    case rivaler::SafetyFault::kWallTooClose:
      return "wall-too-close";
    case rivaler::SafetyFault::kNoFloorDetected:
      return "no-floor";
  }
  return "unknown";
}

void printUltrasonicDiagnostics(unsigned long nowMs) {
  if (!rivaler_giga_config::kUltrasonicDiagnosticLoggingEnabled ||
      ultrasonicSafety.isMeasurementInProgress() ||
      nowMs - lastUltrasonicDiagnosticPrintMs <
          rivaler_giga_config::kUltrasonicDiagnosticLogIntervalMs) {
    return;
  }

  lastUltrasonicDiagnosticPrintMs = nowMs;
  Serial.print("Ultrasonic: front=");
  Serial.print(ultrasonicSafety.frontDistanceCm());
  Serial.print(" cm, down=");
  Serial.print(ultrasonicSafety.groundDistanceCm());
  Serial.print(" cm, wall=");
  Serial.print(ultrasonicSafety.isWallTooClose() ? "STOP" : "clear");
  Serial.print(", floor=");
  Serial.print(ultrasonicSafety.isNoFloorDetected() ? "STOP" : "clear");
  Serial.print(", fault=");
  Serial.println(safetyFaultName(activeSafetyFault()));
}

void printRemoteControlDiagnostics() {
  if (!rivaler_giga_config::kRemoteControlDiagnosticLoggingEnabled ||
      !remoteControlDiagnosticPending ||
      ultrasonicSafety.isMeasurementInProgress()) {
    return;
  }

  remoteControlDiagnosticPending = false;
  Serial.print("Remote control: seq=");
  Serial.print(remoteControlDiagnosticSequence);
  Serial.print(", joystickX=");
  Serial.print(remoteControlDiagnostic.joystickX);
  Serial.print(", joystickY=");
  Serial.print(remoteControlDiagnostic.joystickY);
  Serial.print(", held=0x");
  Serial.print(remoteControlDiagnostic.heldButtons, HEX);
  Serial.print(", events=0x");
  Serial.println(remoteControlDiagnostic.edgeEvents, HEX);
}

void applyStartupDriveTest(unsigned long nowMs) {
  if (!rivaler_giga_config::kStartupDriveTestEnabled ||
      startupDriveTestComplete) {
    return;
  }

  if (nowMs - startupDriveTestStartedMs >=
      rivaler_giga_config::kStartupDriveTestDurationMs) {
    startupDriveTestComplete = true;
    driveController.stop();
    Serial.println("Startup drive test complete.");
    return;
  }

  if (activeSafetyFault() == rivaler::SafetyFault::kNone) {
    driveController.setJoystickCommand(
        0, rivaler_giga_config::kStartupDriveTestPercent);
  }
}

void processRobotCommands(unsigned long nowMs) {
  rivaler::Packet packet{};
  while (robotLink.readPacket(packet)) {
    if (packet.type != rivaler::MessageType::kRemoteControl) {
      continue;
    }

    rivaler::RemoteControlPayload control{};
    if (!rivaler::readPayload(packet, control)) {
      continue;
    }

    remoteControlDiagnostic = control;
    remoteControlDiagnosticSequence = packet.sequence;
    remoteControlDiagnosticPending = true;

    hasReceivedControl = true;
    lastControlReceivedMs = nowMs;
    remoteTimeoutActive = false;
    emergencyStopActive =
        (control.heldButtons & rivaler::kEmergencyStopHeld) != 0;
    launcherArmRequested =
        (control.heldButtons & rivaler::kArmSwitchHeld) != 0;

    const bool safetyStopActive =
        activeSafetyFault() != rivaler::SafetyFault::kNone;
    const bool followLineToggleRequested =
        (control.edgeEvents & rivaler::kFollowLinePressed) != 0;
    if (followLineToggleRequested) {
      if (lineFollower.isActive()) {
        lineFollower.deactivate();
        driveController.stop();
        Serial.println("Line following disabled.");
      } else if (safetyStopActive) {
        Serial.println("Line following rejected: safety stop is active.");
      } else if (lineFollower.activate(lineSensors, nowMs)) {
        driveController.stop();
        Serial.println("Line following enabled.");
      } else if (!lineSensors.isCalibrationValid()) {
        Serial.println("Line following rejected: sensors are not calibrated.");
      } else {
        Serial.println("Line following rejected: position the sensor array over the line.");
      }
    }

    const bool manualOverrideRequested =
        abs(control.joystickX) >=
            rivaler_giga_config::kLineFollowingManualOverridePercent ||
        abs(control.joystickY) >=
            rivaler_giga_config::kLineFollowingManualOverridePercent;
    if (lineFollower.isActive() && !followLineToggleRequested &&
        manualOverrideRequested) {
      lineFollower.deactivate();
      Serial.println("Line following disabled by joystick override.");
    }

    const bool quickShootRequested =
        (control.edgeEvents & rivaler::kQuickShootPressed) != 0;
    const bool rapidFireRequested =
        (control.edgeEvents & rivaler::kRapidFirePressed) != 0;
    if (rapidFireRequested && launcherArmRequested && !safetyStopActive &&
        !launcherController.requestRapidFire(nowMs)) {
      Serial.println("Rapid Fire ignored: launcher is not ready.");
    } else if (quickShootRequested && launcherArmRequested &&
               !safetyStopActive &&
               !launcherController.requestQuickShoot(nowMs)) {
      Serial.println("Quick Shoot ignored: launcher is not ready.");
    }

    if (safetyStopActive) {
      driveController.stop();
      continue;
    }

    if (!lineFollower.isActive()) {
      driveController.setJoystickCommand(control.joystickX, control.joystickY);
    }
  }
}

void printLineSensorCalibration(unsigned long nowMs) {
  if (!rivaler_giga_config::kLineSensorCalibrationLoggingEnabled ||
      nowMs - lastLineCalibrationPrintMs <
          rivaler_giga_config::kLineSensorCalibrationLogIntervalMs) {
    return;
  }

  lastLineCalibrationPrintMs = nowMs;
  const bool calibrationValid = lineSensors.isCalibrationValid();
  Serial.print("Left: analog=");
  Serial.print(lineSensors.leftReading());
  Serial.print(", line=");
  if (calibrationValid) {
    Serial.print(lineSensors.isLeftOnLine() ? "YES" : "NO");
  } else {
    Serial.print("UNCALIBRATED");
  }
  Serial.print(" | Right: analog=");
  Serial.print(lineSensors.rightReading());
  Serial.print(", line=");
  if (calibrationValid) {
    Serial.println(lineSensors.isRightOnLine() ? "YES" : "NO");
  } else {
    Serial.println("UNCALIBRATED");
  }
}

void applyControlTimeout(unsigned long nowMs) {
  const bool timedOut = hasReceivedControl &&
                        nowMs - lastControlReceivedMs >
                            rivaler_giga_config::kRemoteCommandTimeoutMs;
  if (!timedOut) {
    return;
  }

  if (!remoteTimeoutActive) {
    Serial.println("Remote command timeout: drive stopped.");
  }
  remoteTimeoutActive = true;
  emergencyStopActive = false;
  launcherArmRequested = false;
  driveController.stop();
}

void sendRobotStatus(unsigned long nowMs) {
  if (nowMs - lastStatusTransmitMs <
      rivaler_giga_config::kRobotStatusTransmitIntervalMs) {
    return;
  }

  rivaler::RobotStatusPayload status{};
  status.frontDistanceCm = ultrasonicSafety.frontDistanceCm();
  status.groundDistanceCm = ultrasonicSafety.groundDistanceCm();
  if (lineSensors.isLineDetected()) {
    status.statusFlags |= rivaler::kLineDetected;
  }
  if (lineFollower.isActive()) {
    status.statusFlags |= rivaler::kLineFollowingActive;
  }
  if (launcherController.isArmed()) {
    status.statusFlags |= rivaler::kLauncherArmed;
  }

  const rivaler::SafetyFault safetyFault = activeSafetyFault();
  if (safetyFault == rivaler::SafetyFault::kRemoteTimeout) {
    status.statusFlags |= rivaler::kRemoteLinkTimedOut;
    status.statusFlags |= rivaler::kSafetyWarning;
    status.safetyFault = safetyFault;
  } else if (safetyFault != rivaler::SafetyFault::kNone) {
    status.statusFlags |= rivaler::kSafetyWarning;
    status.safetyFault = safetyFault;
  }

  rivaler::Packet packet{};
  rivaler::initializePacket(packet, rivaler::MessageType::kRobotStatus,
                            nextStatusSequence++);
  if (rivaler::setPayload(packet, status)) {
    robotLink.sendPacket(packet);
  }

  lastStatusTransmitMs = nowMs;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Rivaler GIGA controller starting.");

  robotLink.begin(rivaler_giga_config::kRobotNanoUartBaud);
  driveController.begin();
  startupDriveTestStartedMs = millis();
  launcherController.begin();
  lineSensors.begin();
  ultrasonicSafety.begin();

  if (!lineSensors.isCalibrationValid()) {
    Serial.println("Line following locked: sensor calibration is required.");
  }
}

void loop() {
  const unsigned long nowMs = millis();
  ultrasonicSafety.update(micros());
  lineSensors.update(nowMs);
  printUltrasonicDiagnostics(nowMs);
  printRemoteControlDiagnostics();
  printLineSensorCalibration(nowMs);
  processRobotCommands(nowMs);
  applyControlTimeout(nowMs);
  applyStartupDriveTest(nowMs);
  const bool safetyStopActive =
      activeSafetyFault() != rivaler::SafetyFault::kNone;
  if (safetyStopActive) {
    lineFollower.deactivate();
    driveController.stop();
  }
  launcherController.update(launcherArmRequested,
                            safetyStopActive, nowMs);
  if (!safetyStopActive) {
    lineFollower.update(lineSensors, driveController, nowMs);
  }
  driveController.update(micros());
  sendRobotStatus(nowMs);
}
