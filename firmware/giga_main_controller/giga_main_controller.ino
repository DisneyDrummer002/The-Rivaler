#include "Config.h"
#include "DriveController.h"
#include "GigaCommandReceiver.h"
#include "LauncherController.h"
#include "LineSensors.h"
#include "UltrasonicSafety.h"

namespace {

GigaCommandReceiver robotLink(Serial3);
DriveController driveController;
LauncherController launcherController;
LineSensors lineSensors;
UltrasonicSafety ultrasonicSafety;

bool hasReceivedControl = false;
bool emergencyStopActive = false;
bool remoteTimeoutActive = false;
bool launcherArmRequested = false;
unsigned long lastControlReceivedMs = 0;
unsigned long lastStatusTransmitMs = 0;
uint8_t nextStatusSequence = 0;

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

    hasReceivedControl = true;
    lastControlReceivedMs = nowMs;
    remoteTimeoutActive = false;
    emergencyStopActive =
        (control.heldButtons & rivaler::kEmergencyStopHeld) != 0;
    launcherArmRequested =
        (control.heldButtons & rivaler::kArmSwitchHeld) != 0;

    const bool quickShootRequested =
        (control.edgeEvents & rivaler::kQuickShootPressed) != 0;
    const bool rapidFireRequested =
        (control.edgeEvents & rivaler::kRapidFirePressed) != 0;
    if (rapidFireRequested && launcherArmRequested && !emergencyStopActive &&
        !launcherController.requestRapidFire(nowMs)) {
      Serial.println("Rapid Fire ignored: launcher is not ready.");
    } else if (quickShootRequested && launcherArmRequested &&
               !emergencyStopActive &&
               !launcherController.requestQuickShoot(nowMs)) {
      Serial.println("Quick Shoot ignored: launcher is not ready.");
    }

    if (emergencyStopActive) {
      driveController.stop();
      continue;
    }

    driveController.setJoystickCommand(control.joystickX, control.joystickY);
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
  if (launcherController.isArmed()) {
    status.statusFlags |= rivaler::kLauncherArmed;
  }

  if (remoteTimeoutActive) {
    status.statusFlags |= rivaler::kRemoteLinkTimedOut;
    status.statusFlags |= rivaler::kSafetyWarning;
    status.safetyFault = rivaler::SafetyFault::kRemoteTimeout;
  } else if (emergencyStopActive) {
    status.statusFlags |= rivaler::kSafetyWarning;
    status.safetyFault = rivaler::SafetyFault::kEmergencyStop;
  } else if (ultrasonicSafety.isNoFloorDetected()) {
    status.statusFlags |= rivaler::kSafetyWarning;
    status.safetyFault = rivaler::SafetyFault::kNoFloorDetected;
  } else if (ultrasonicSafety.isWallTooClose()) {
    status.statusFlags |= rivaler::kSafetyWarning;
    status.safetyFault = rivaler::SafetyFault::kWallTooClose;
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
  launcherController.begin();
  lineSensors.begin();
  ultrasonicSafety.begin();
}

void loop() {
  const unsigned long nowMs = millis();
  processRobotCommands(nowMs);
  applyControlTimeout(nowMs);
  launcherController.update(launcherArmRequested,
                            remoteTimeoutActive || emergencyStopActive,
                            nowMs);
  driveController.update(micros());
  lineSensors.update(nowMs);
  ultrasonicSafety.update(micros());
  sendRobotStatus(nowMs);
}
