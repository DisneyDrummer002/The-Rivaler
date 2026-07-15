#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <esp_idf_version.h>

#include "RivalerProtocol.h"

class EspNowRemote {
 public:
  bool begin();
  bool sendRemoteControl(const rivaler::Packet& packet);
  bool takeLatestRobotStatus(rivaler::RobotStatusPayload& status);
  bool takeLatestAcknowledgement(rivaler::AcknowledgementPayload& acknowledgement);
  bool hasHealthyStatusLink(unsigned long nowMs) const;

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

  static EspNowRemote* instance_;
  static portMUX_TYPE statusMutex_;

  rivaler::RobotStatusPayload latestStatus_{};
  rivaler::AcknowledgementPayload latestAcknowledgement_{};
  bool statusAvailable_ = false;
  bool acknowledgementAvailable_ = false;
  bool hasReceivedStatus_ = false;
  unsigned long lastStatusReceivedMs_ = 0;
};
