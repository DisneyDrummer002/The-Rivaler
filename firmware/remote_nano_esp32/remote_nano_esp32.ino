#include "Config.h"
#include "EspNowRemote.h"
#include "RemoteInputs.h"
#include "StatusOutputs.h"

namespace {

RemoteInputs remoteInputs;
EspNowRemote espNowRemote;
StatusOutputs statusOutputs;

uint8_t nextSequence = 0;
unsigned long lastControlTransmitMs = 0;
unsigned long lastDebugPrintMs = 0;
bool radioReady = false;

void sendCurrentControl(unsigned long nowMs) {
  rivaler::RemoteControlPayload control = remoteInputs.createControlPayload();
  rivaler::Packet packet{};
  rivaler::initializePacket(packet, rivaler::MessageType::kRemoteControl,
                            nextSequence++);
  if (!rivaler::setPayload(packet, control)) {
    return;
  }

  if (espNowRemote.sendRemoteControl(packet)) {
    remoteInputs.markEventsQueued(control.edgeEvents);
  }

  lastControlTransmitMs = nowMs;
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
  statusOutputs.begin();
  radioReady = espNowRemote.begin();
}

void loop() {
  const unsigned long nowMs = millis();
  remoteInputs.update(nowMs);

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
