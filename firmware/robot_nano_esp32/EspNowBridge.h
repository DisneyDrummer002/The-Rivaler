#pragma once

#include <Arduino.h>
#include <esp_idf_version.h>
#include <esp_now.h>

#include <RivalerProtocol.h>

class EspNowBridge {
 public:
  bool begin();
  bool takeLatestRemoteControl(rivaler::Packet& packet);
  bool sendRobotStatus(const rivaler::Packet& packet);
  bool hasReceivedRemoteControl() const;
  bool hasHealthyRemoteLink(unsigned long nowMs) const;

 private:
  static void onSend(const uint8_t* macAddress, esp_now_send_status_t status);

#if ESP_IDF_VERSION_MAJOR >= 5
  static void onReceive(const esp_now_recv_info_t* receiveInfo,
                        const uint8_t* data, int dataLength);
#else
  static void onReceive(const uint8_t* macAddress, const uint8_t* data,
                        int dataLength);
#endif

  void handleReceive(const uint8_t* macAddress, const uint8_t* data,
                     int dataLength);

  static EspNowBridge* instance_;
  static portMUX_TYPE packetMutex_;

  rivaler::Packet latestRemoteControl_{};
  bool remoteControlAvailable_ = false;
  bool hasReceivedRemoteControl_ = false;
  unsigned long lastRemoteControlMs_ = 0;
};
