#include "EspNowBridge.h"

#include <WiFi.h>

#include "Config.h"

using namespace rivaler_robot_nano_config;

EspNowBridge* EspNowBridge::instance_ = nullptr;
portMUX_TYPE EspNowBridge::packetMutex_ = portMUX_INITIALIZER_UNLOCKED;

bool EspNowBridge::begin() {
  instance_ = this;
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW initialization failed.");
    return false;
  }

  if (esp_now_register_send_cb(onSend) != ESP_OK ||
      esp_now_register_recv_cb(onReceive) != ESP_OK) {
    Serial.println("ESP-NOW callback registration failed.");
    return false;
  }

  esp_now_peer_info_t peerInfo{};
  memcpy(peerInfo.peer_addr, kRemoteNanoMac, sizeof(kRemoteNanoMac));
  peerInfo.channel = kEspNowChannel;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(kRemoteNanoMac) &&
      esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-NOW peer registration failed.");
    return false;
  }

  Serial.print("Robot ESP-NOW MAC: ");
  Serial.println(WiFi.macAddress());
  return true;
}

bool EspNowBridge::takeLatestRemoteControl(rivaler::Packet& packet) {
  portENTER_CRITICAL(&packetMutex_);
  const bool hasPacket = remoteControlAvailable_;
  if (hasPacket) {
    packet = latestRemoteControl_;
    remoteControlAvailable_ = false;
  }
  portEXIT_CRITICAL(&packetMutex_);
  return hasPacket;
}

bool EspNowBridge::sendRobotStatus(const rivaler::Packet& packet) {
  if (packet.type != rivaler::MessageType::kRobotStatus ||
      !rivaler::isPacketValid(packet)) {
    return false;
  }

  return sendPacket(packet);
}

bool EspNowBridge::sendPendingAcknowledgement() {
  rivaler::Packet acknowledgement{};
  portENTER_CRITICAL(&packetMutex_);
  const bool hasAcknowledgement = acknowledgementPending_;
  if (hasAcknowledgement) {
    acknowledgement = pendingAcknowledgement_;
    acknowledgementPending_ = false;
  }
  portEXIT_CRITICAL(&packetMutex_);

  return !hasAcknowledgement || sendPacket(acknowledgement);
}

bool EspNowBridge::sendPacket(const rivaler::Packet& packet) {
  uint8_t wireBytes[rivaler::kMaxPacketBytes]{};
  if (!rivaler::encodePacket(packet, wireBytes, sizeof(wireBytes))) {
    return false;
  }

  return esp_now_send(kRemoteNanoMac,
                      wireBytes,
                      rivaler::packetWireSize(packet)) == ESP_OK;
}

bool EspNowBridge::hasReceivedRemoteControl() const {
  portENTER_CRITICAL(&packetMutex_);
  const bool hasReceived = hasReceivedRemoteControl_;
  portEXIT_CRITICAL(&packetMutex_);
  return hasReceived;
}

bool EspNowBridge::hasHealthyRemoteLink(unsigned long nowMs) const {
  portENTER_CRITICAL(&packetMutex_);
  const bool healthy = hasReceivedRemoteControl_ &&
                       nowMs - lastRemoteControlMs_ <=
                           kRemoteCommandTimeoutMs;
  portEXIT_CRITICAL(&packetMutex_);
  return healthy;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void EspNowBridge::onSend(const esp_now_send_info_t* sendInfo,
                           esp_now_send_status_t status) {
  (void)sendInfo;
  (void)status;
}
#else
void EspNowBridge::onSend(const uint8_t* macAddress,
                           esp_now_send_status_t status) {
  (void)macAddress;
  (void)status;
}
#endif

#if ESP_IDF_VERSION_MAJOR >= 5
void EspNowBridge::onReceive(const esp_now_recv_info_t* receiveInfo,
                              const uint8_t* data, int dataLength) {
  if (receiveInfo != nullptr && instance_ != nullptr) {
    instance_->handleReceive(receiveInfo->src_addr, data, dataLength);
  }
}
#else
void EspNowBridge::onReceive(const uint8_t* macAddress, const uint8_t* data,
                              int dataLength) {
  if (instance_ != nullptr) {
    instance_->handleReceive(macAddress, data, dataLength);
  }
}
#endif

void EspNowBridge::handleReceive(const uint8_t* macAddress,
                                  const uint8_t* data, int dataLength) {
  if (macAddress == nullptr || data == nullptr ||
      dataLength < rivaler::kPacketHeaderBytes + rivaler::kPacketChecksumBytes ||
      dataLength > rivaler::kMaxPacketBytes ||
      memcmp(macAddress, kRemoteNanoMac, sizeof(kRemoteNanoMac)) != 0) {
    return;
  }

  rivaler::Packet receivedPacket{};
  if (!rivaler::decodePacket(data, static_cast<uint8_t>(dataLength),
                             receivedPacket) ||
      receivedPacket.type != rivaler::MessageType::kRemoteControl) {
    return;
  }

  rivaler::RemoteControlPayload receivedControl{};
  if (!rivaler::readPayload(receivedPacket, receivedControl)) {
    return;
  }

  portENTER_CRITICAL(&packetMutex_);
  const bool duplicate = hasLastRemoteControlSequence_ &&
                         receivedPacket.sequence == lastRemoteControlSequence_;
  if (receivedControl.edgeEvents != 0) {
    rivaler::AcknowledgementPayload acknowledgementPayload{};
    acknowledgementPayload.acknowledgedSequence = receivedPacket.sequence;
    acknowledgementPayload.accepted = true;
    acknowledgementPayload.rejectionReason = rivaler::SafetyFault::kNone;
    rivaler::initializePacket(pendingAcknowledgement_,
                              rivaler::MessageType::kAcknowledgement,
                              receivedPacket.sequence);
    acknowledgementPending_ = rivaler::setPayload(pendingAcknowledgement_,
                                                   acknowledgementPayload);
  }

  hasReceivedRemoteControl_ = true;
  lastRemoteControlMs_ = millis();
  if (duplicate) {
    portEXIT_CRITICAL(&packetMutex_);
    return;
  }

  if (remoteControlAvailable_) {
    rivaler::RemoteControlPayload pendingControl{};
    if (rivaler::readPayload(latestRemoteControl_, pendingControl)) {
      receivedControl.edgeEvents |= pendingControl.edgeEvents;
      rivaler::setPayload(receivedPacket, receivedControl);
    }
  }

  latestRemoteControl_ = receivedPacket;
  remoteControlAvailable_ = true;
  hasLastRemoteControlSequence_ = true;
  lastRemoteControlSequence_ = receivedPacket.sequence;
  portEXIT_CRITICAL(&packetMutex_);
}
