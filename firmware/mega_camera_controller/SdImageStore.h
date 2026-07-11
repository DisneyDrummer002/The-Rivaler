#pragma once

#include <Arduino.h>
#include <SD.h>

class SdImageStore {
 public:
  SdImageStore(uint8_t cameraChipSelectPin, uint8_t sdChipSelectPin);

  bool begin();
  void end();
  bool isReady() const;
  File openNextImage(char* filename, size_t filenameLength);

 private:
  void deselectCamera() const;
  void deselectSd() const;

  uint8_t cameraChipSelectPin_;
  uint8_t sdChipSelectPin_;
  bool ready_;
  unsigned long nextImageNumber_;
};
