#include "Config.h"
#include "EspNowBridge.h"
#include "GigaSerialBridge.h"
#include "MegaCameraCommand.h"

namespace {

HardwareSerial gigaUart(rivaler_robot_nano_config::kGigaUartPort);
HardwareSerial megaUart(rivaler_robot_nano_config::kMegaUartPort);
EspNowBridge espNowBridge;
GigaSerialBridge gigaBridge(gigaUart);
MegaCameraCommand megaCamera(megaUart);

bool espNowReady = false;
bool remoteTimeoutStopSent = false;
uint8_t nextCameraSequence = 0;
uint8_t nextFailsafeSequence = 0;

void forwardPendingRemoteControl() {
  rivaler::Packet packet{};
  if (!espNowBridge.takeLatestRemoteControl(packet)) {
    return;
  }

  rivaler::RemoteControlPayload control{};
  if (!rivaler::readPayload(packet, control)) {
    return;
  }

  gigaBridge.sendPacket(packet);
  megaCamera.sendForRemoteControl(control, nextCameraSequence);
}

void relayGigaStatuses() {
  rivaler::Packet packet{};
  while (gigaBridge.readPacket(packet)) {
    if (packet.type == rivaler::MessageType::kRobotStatus) {
      espNowBridge.sendRobotStatus(packet);
    }
  }
}

void sendRemoteTimeoutStop(unsigned long nowMs) {
  if (!espNowBridge.hasReceivedRemoteControl() ||
      espNowBridge.hasHealthyRemoteLink(nowMs)) {
    remoteTimeoutStopSent = false;
    return;
  }

  if (remoteTimeoutStopSent) {
    return;
  }

  rivaler::RemoteControlPayload safeStop{};
  rivaler::Packet packet{};
  rivaler::initializePacket(packet, rivaler::MessageType::kRemoteControl,
                            nextFailsafeSequence++);
  if (rivaler::setPayload(packet, safeStop) && gigaBridge.sendPacket(packet)) {
    remoteTimeoutStopSent = true;
    Serial.println("Remote timeout: safe stop forwarded to GIGA.");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Rivaler robot bridge starting.");

  gigaBridge.begin(rivaler_robot_nano_config::kGigaUartBaud,
                   rivaler_robot_nano_config::kGigaUartRxPin,
                   rivaler_robot_nano_config::kGigaUartTxPin);
  megaCamera.begin(rivaler_robot_nano_config::kMegaUartBaud,
                   rivaler_robot_nano_config::kMegaUartTxPin);
  espNowReady = espNowBridge.begin();
}

void loop() {
  const unsigned long nowMs = millis();

  if (espNowReady) {
    forwardPendingRemoteControl();
    sendRemoteTimeoutStop(nowMs);
  }

  relayGigaStatuses();
}
