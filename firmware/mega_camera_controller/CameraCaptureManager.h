#pragma once

#include <Arduino.h>
#include <Arducam_Mega.h>
#include <SD.h>

#include "SdImageStore.h"

class CameraCaptureManager {
 public:
  CameraCaptureManager(uint8_t cameraChipSelectPin, uint8_t sdChipSelectPin,
                       SdImageStore& imageStore);

  void begin();
  bool requestBurst(uint8_t photoCount, unsigned long intervalMs,
                    unsigned long now);
  bool requestSafeEject();
  void update(unsigned long now);

  bool isBusy() const;
  bool isReady() const;
  bool isSdSafeToRemove() const;

 private:
  enum class State : uint8_t { kIdle, kWaitingForNextImage, kReadingImage };

  void startNextImage(unsigned long now);
  void processImageBytes(unsigned long now);
  void finishImage(unsigned long now);
  void finishSafeEject();
  void abortBurst(const __FlashStringHelper* reason);
  void closeCurrentFile();
  void deselectCamera() const;
  void deselectSd() const;

  static constexpr uint8_t kBytesPerUpdate = 96;
  static constexpr unsigned long kImageReceiveTimeoutMs = 5000;

  uint8_t cameraChipSelectPin_;
  uint8_t sdChipSelectPin_;
  SdImageStore& imageStore_;
  Arducam_Mega camera_;
  State state_;
  bool cameraReady_;
  bool safeEjectPending_;
  bool sdSafeToRemove_;
  uint8_t photosRemaining_;
  unsigned long intervalMs_;
  unsigned long nextImageAt_;
  unsigned long imageStartedAt_;
  unsigned long lastDataAt_;
  unsigned long imageByteCount_;
  uint8_t previousByte_;
  bool jpegStarted_;
  File currentFile_;
  char currentFilename_[12];
};
