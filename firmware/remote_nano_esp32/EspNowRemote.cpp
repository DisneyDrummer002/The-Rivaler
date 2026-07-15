#include "EspNowRemote.h"

#include <WiFi.h>
#include <esp_idf_version.h>

#include "Config.h"

using namespace rivaler_remote_config;

EspNowRemote* EspNowRemote::instance_ = nullptr;
portMUX_TYPE EspNowRemote::statusMutex_ = portMUX_INITIALIZER_UNLOCKED;

bool EspNowRemote::begin() {
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
  memcpy(peerInfo.peer_addr, kRobotNanoMac, sizeof(kRobotNanoMac));
  peerInfo.channel = kEspNowChannel;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(kRobotNanoMac) &&
      esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ESP-NOW peer registration failed.");
    return false;
  }

  Serial.print("Remote ESP-NOW MAC: ");
  Serial.println(WiFi.macAddress());
  return true;
}

bool EspNowRemote::sendRemoteControl(const rivaler::Packet& packet) {
  if (packet.type != rivaler::MessageType::kRemoteControl ||
      !rivaler::isPacketValid(packet)) {
    return false;
  }

  uint8_t wireBytes[rivaler::kMaxPacketBytes]{};
  if (!rivaler::encodePacket(packet, wireBytes, sizeof(wireBytes))) {
    return false;
  }

  return esp_now_send(kRobotNanoMac,
                      wireBytes,
                      rivaler::packetWireSize(packet)) == ESP_OK;
}

bool EspNowRemote::takeLatestRobotStatus(
    rivaler::RobotStatusPayload& status) {
  portENTER_CRITICAL(&statusMutex_);
  const bool hasStatus = statusAvailable_;
  if (hasStatus) {
    status = latestStatus_;
    statusAvailable_ = false;
  }
  portEXIT_CRITICAL(&statusMutex_);
  return hasStatus;
}

bool EspNowRemote::takeLatestAcknowledgement(
    rivaler::AcknowledgementPayload& acknowledgement) {
  portENTER_CRITICAL(&statusMutex_);
  const bool hasAcknowledgement = acknowledgementAvailable_;
  if (hasAcknowledgement) {
    acknowledgement = latestAcknowledgement_;
    acknowledgementAvailable_ = false;
  }
  portEXIT_CRITICAL(&statusMutex_);
  return hasAcknowledgement;
}

bool EspNowRemote::hasHealthyStatusLink(unsigned long nowMs) const {
  portENTER_CRITICAL(&statusMutex_);
  const bool healthy = hasReceivedStatus_ &&
                       nowMs - lastStatusReceivedMs_ <=
                           kEspNowStatusTimeoutMs;
  portEXIT_CRITICAL(&statusMutex_);
  return healthy;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void EspNowRemote::onSend(const esp_now_send_info_t* sendInfo,
                           esp_now_send_status_t status) {
  (void)sendInfo;
  (void)status;
}
#else
void EspNowRemote::onSend(const uint8_t* macAddress,
                           esp_now_send_status_t status) {
  (void)macAddress;
  (void)status;
}
#endif

#if ESP_IDF_VERSION_MAJOR >= 5
void EspNowRemote::onReceive(const esp_now_recv_info_t* receiveInfo,
                              const uint8_t* data, int dataLength) {
  if (receiveInfo != nullptr && instance_ != nullptr) {
    instance_->handleReceive(receiveInfo->src_addr, data, dataLength);
  }
}
#else
void EspNowRemote::onReceive(const uint8_t* macAddress, const uint8_t* data,
                              int dataLength) {
  if (instance_ != nullptr) {
    instance_->handleReceive(macAddress, data, dataLength);
  }
}
#endif

void EspNowRemote::handleReceive(const uint8_t* macAddress,
                                  const uint8_t* data, int dataLength) {
  if (macAddress == nullptr || data == nullptr ||
      dataLength < rivaler::kPacketHeaderBytes + rivaler::kPacketChecksumBytes ||
      dataLength > rivaler::kMaxPacketBytes ||
      memcmp(macAddress, kRobotNanoMac, sizeof(kRobotNanoMac)) != 0) {
    return;
  }

  rivaler::Packet packet{};
  if (!rivaler::decodePacket(data, static_cast<uint8_t>(dataLength), packet)) {
    return;
  }

  if (packet.type == rivaler::MessageType::kAcknowledgement) {
    rivaler::AcknowledgementPayload acknowledgement{};
    if (!rivaler::readPayload(packet, acknowledgement)) {
      return;
    }

    portENTER_CRITICAL(&statusMutex_);
    latestAcknowledgement_ = acknowledgement;
    acknowledgementAvailable_ = true;
    portEXIT_CRITICAL(&statusMutex_);
    return;
  }

  if (packet.type != rivaler::MessageType::kRobotStatus) {
    return;
  }

  rivaler::RobotStatusPayload status{};
  if (!rivaler::readPayload(packet, status)) {
    return;
  }

  portENTER_CRITICAL(&statusMutex_);
  latestStatus_ = status;
  statusAvailable_ = true;
  hasReceivedStatus_ = true;
  lastStatusReceivedMs_ = millis();
  portEXIT_CRITICAL(&statusMutex_);
}
