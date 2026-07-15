#pragma once

#include <Arduino.h>
#include <esp_idf_version.h>
#include <esp_now.h>

#include "RivalerProtocol.h"

class EspNowBridge {
 public:
  bool begin();
  bool takeLatestRemoteControl(rivaler::Packet& packet);
  bool sendRobotStatus(const rivaler::Packet& packet);
  bool sendPendingAcknowledgement();
  bool hasReceivedRemoteControl() const;
  bool hasHealthyRemoteLink(unsigned long nowMs) const;

 private:
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  static void onSend(const esp_now_send_info_t* sendInfo,
                     esp_now_send_status_t status);
#else
  static void onSend(const uint8_t* macAddress, esp_now_send_status_t status);
#endif

#if ESP_IDF_VERSION_MAJOR >= 5
  static void onReceive(const esp_now_recv_info_t* receiveInfo,
                        const uint8_t* data, int dataLength);
#else
  static void onReceive(const uint8_t* macAddress, const uint8_t* data,
                        int dataLength);
#endif

  void handleReceive(const uint8_t* macAddress, const uint8_t* data,
                     int dataLength);
  bool sendPacket(const rivaler::Packet& packet);

  static EspNowBridge* instance_;
  static portMUX_TYPE packetMutex_;

  rivaler::Packet latestRemoteControl_{};
  rivaler::Packet pendingAcknowledgement_{};
  bool remoteControlAvailable_ = false;
  bool acknowledgementPending_ = false;
  bool hasReceivedRemoteControl_ = false;
  bool hasLastRemoteControlSequence_ = false;
  uint8_t lastRemoteControlSequence_ = 0;
  unsigned long lastRemoteControlMs_ = 0;
};
