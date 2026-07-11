#include "SdImageStore.h"

#include <SPI.h>
#include <stdio.h>
#include <string.h>

SdImageStore::SdImageStore(uint8_t cameraChipSelectPin, uint8_t sdChipSelectPin)
    : cameraChipSelectPin_(cameraChipSelectPin),
      sdChipSelectPin_(sdChipSelectPin),
      ready_(false),
      nextImageNumber_(1) {}

bool SdImageStore::begin() {
  pinMode(cameraChipSelectPin_, OUTPUT);
  pinMode(sdChipSelectPin_, OUTPUT);
  deselectCamera();
  deselectSd();
  SPI.begin();

  // SD.begin selects the card internally. Keep the camera inactive throughout.
  deselectCamera();
  ready_ = SD.begin(sdChipSelectPin_);
  deselectCamera();
  deselectSd();
  return ready_;
}

void SdImageStore::end() {
  if (!ready_) {
    return;
  }

  // No file remains open when this is called. End the SD session before the
  // card is removed, then leave both SPI devices inactive.
  deselectCamera();
  SD.end();
  ready_ = false;
  deselectCamera();
  deselectSd();
}

bool SdImageStore::isReady() const { return ready_; }

File SdImageStore::openNextImage(char* filename, size_t filenameLength) {
  if (!ready_ || filenameLength < 12) {
    return File();
  }

  for (unsigned long attempts = 0; attempts < 99999; ++attempts) {
    const unsigned long candidate = nextImageNumber_;
    nextImageNumber_ = candidate >= 99999 ? 1 : candidate + 1;

    char candidateName[12] = {};
    snprintf(candidateName, sizeof(candidateName), "RV%05lu.JPG", candidate);
    deselectCamera();
    if (SD.exists(candidateName)) {
      continue;
    }

    File image = SD.open(candidateName, FILE_WRITE);
    deselectCamera();
    deselectSd();
    if (!image) {
      return File();
    }

    strncpy(filename, candidateName, filenameLength);
    filename[filenameLength - 1] = '\0';
    return image;
  }

  return File();
}

void SdImageStore::deselectCamera() const {
  digitalWrite(cameraChipSelectPin_, HIGH);
}

void SdImageStore::deselectSd() const { digitalWrite(sdChipSelectPin_, HIGH); }
