#include "Config.h"
#include "EspNowRemote.h"
#include "RemoteInputs.h"
#include "StatusOutputs.h"

namespace {

RemoteInputs remoteInputs;
EspNowRemote espNowRemote;
StatusOutputs statusOutputs;

uint8_t nextSequence = 0;
bool eventAcknowledgementPending = false;
uint8_t pendingEventSequence = 0;
uint16_t pendingEventMask = 0;
unsigned long lastControlTransmitMs = 0;
unsigned long lastDebugPrintMs = 0;
bool radioReady = false;
uint16_t lastReportedHeldButtons = 0;

void printEventNames(uint16_t events) {
  bool needsSeparator = false;
  const auto printName = [&needsSeparator](const char* name) {
    if (needsSeparator) {
      Serial.print(", ");
    }
    Serial.print(name);
    needsSeparator = true;
  };

  if ((events & rivaler::kQuickShootPressed) != 0) {
    printName("Quick Shoot");
  }
  if ((events & rivaler::kRapidFirePressed) != 0) {
    printName("Rapid Fire");
  }
  if ((events & rivaler::kTakePicturePressed) != 0) {
    printName("Take Picture");
  }
  if ((events & rivaler::kEjectSdPressed) != 0) {
    printName("SD Eject");
  }
  if ((events & rivaler::kFollowLinePressed) != 0) {
    printName("Follow Line");
  }
}

void printHeldButtonChanges(uint16_t previousButtons, uint16_t currentButtons) {
  const uint16_t changedButtons = previousButtons ^ currentButtons;
  if (changedButtons == 0) {
    return;
  }

  Serial.print("Input: ");
  bool needsSeparator = false;
  const auto printChange = [&currentButtons, &needsSeparator](uint16_t bit,
                                                                const char* name) {
    if (needsSeparator) {
      Serial.print(", ");
    }
    Serial.print(name);
    Serial.print((currentButtons & bit) != 0 ? " ON" : " OFF");
    needsSeparator = true;
  };

  if ((changedButtons & rivaler::kArmSwitchHeld) != 0) {
    printChange(rivaler::kArmSwitchHeld, "Arm switch");
  }
  if ((changedButtons & rivaler::kFollowLineHeld) != 0) {
    printChange(rivaler::kFollowLineHeld, "Follow Line");
  }
  if ((changedButtons & rivaler::kEmergencyStopHeld) != 0) {
    printChange(rivaler::kEmergencyStopHeld, "Emergency stop");
  }
  Serial.println();
}

const char* safetyFaultName(rivaler::SafetyFault fault) {
  switch (fault) {
    case rivaler::SafetyFault::kRemoteTimeout:
      return "remote timeout";
    case rivaler::SafetyFault::kWallTooClose:
      return "wall too close";
    case rivaler::SafetyFault::kNoFloorDetected:
      return "no floor detected";
    case rivaler::SafetyFault::kEmergencyStop:
      return "emergency stop";
    case rivaler::SafetyFault::kInvalidCommand:
      return "invalid command";
    case rivaler::SafetyFault::kNone:
    default:
      return "no fault reported";
  }
}

void sendCurrentControl(unsigned long nowMs) {
  uint8_t sequence = 0;
  uint16_t events = 0;
  bool isNewEventPacket = false;
  if (eventAcknowledgementPending) {
    sequence = pendingEventSequence;
    events = pendingEventMask;
  } else {
    sequence = nextSequence++;
    if (remoteInputs.pendingEvents() != 0) {
      events = remoteInputs.pendingEvents();
      pendingEventMask = events;
      pendingEventSequence = sequence;
      eventAcknowledgementPending = true;
      isNewEventPacket = true;
    }
  }

  rivaler::RemoteControlPayload control =
      remoteInputs.createControlPayload(events);
  rivaler::Packet packet{};
  rivaler::initializePacket(packet, rivaler::MessageType::kRemoteControl,
                            sequence);
  if (!rivaler::setPayload(packet, control)) {
    return;
  }

  const uint16_t changedHeldButtons =
      control.heldButtons ^ lastReportedHeldButtons;
  if (changedHeldButtons != 0) {
    printHeldButtonChanges(lastReportedHeldButtons, control.heldButtons);
    lastReportedHeldButtons = control.heldButtons;
  }

  if (isNewEventPacket) {
    Serial.print("Input: ");
    printEventNames(events);
    Serial.println(" pressed.");
  }

  const bool queued = espNowRemote.sendRemoteControl(packet);
  if (isNewEventPacket || changedHeldButtons != 0) {
    Serial.print("ESP-NOW packet ");
    Serial.print(sequence);
    Serial.println(queued ? " queued for transmission."
                          : " was not queued; control sending will retry.");
    if (queued && isNewEventPacket) {
      Serial.println("Waiting for Robot Nano acknowledgement.");
    }
  }

  lastControlTransmitMs = nowMs;
}

void processAcknowledgements() {
  rivaler::AcknowledgementPayload acknowledgement{};
  while (espNowRemote.takeLatestAcknowledgement(acknowledgement)) {
    if (!eventAcknowledgementPending ||
        acknowledgement.acknowledgedSequence != pendingEventSequence) {
      continue;
    }

    Serial.print("Robot Nano acknowledgement for ");
    printEventNames(pendingEventMask);
    if (!acknowledgement.accepted) {
      Serial.print(" rejected: ");
      Serial.println(safetyFaultName(acknowledgement.rejectionReason));
      continue;
    }
    Serial.println(" received.");

    remoteInputs.acknowledgeEvents(pendingEventMask);
    pendingEventMask = 0;
    eventAcknowledgementPending = false;
  }
}

void printDebugStatus(unsigned long nowMs) {
  if (nowMs - lastDebugPrintMs < 1000) {
    return;
  }

  lastDebugPrintMs = nowMs;
  Serial.print("ESP-NOW status link: ");
  Serial.println(espNowRemote.hasHealthyStatusLink(nowMs) ? "healthy"
                                                          : "waiting");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Rivaler remote starting.");

  remoteInputs.begin();
  lastReportedHeldButtons = remoteInputs.createControlPayload(0).heldButtons;
  statusOutputs.begin();
  radioReady = espNowRemote.begin();
}

void loop() {
  const unsigned long nowMs = millis();
  remoteInputs.update(nowMs);
  processAcknowledgements();

  rivaler::RobotStatusPayload status{};
  if (espNowRemote.takeLatestRobotStatus(status)) {
    statusOutputs.applyRobotStatus(status);
  }
  statusOutputs.setStatusLinkHealthy(
      radioReady && espNowRemote.hasHealthyStatusLink(nowMs));
  statusOutputs.update(nowMs);

  if (radioReady && nowMs - lastControlTransmitMs >=
                        rivaler_remote_config::kControlTransmitIntervalMs) {
    sendCurrentControl(nowMs);
  }

  printDebugStatus(nowMs);
}
