#include "CameraCaptureManager.h"

#include <string.h>

CameraCaptureManager::CameraCaptureManager(uint8_t cameraChipSelectPin,
                                           uint8_t sdChipSelectPin,
                                           SdImageStore& imageStore)
    : cameraChipSelectPin_(cameraChipSelectPin),
      sdChipSelectPin_(sdChipSelectPin),
      imageStore_(imageStore),
      camera_(cameraChipSelectPin),
      state_(State::kIdle),
      cameraReady_(false),
      safeEjectPending_(false),
      sdSafeToRemove_(false),
      photosRemaining_(0),
      intervalMs_(0),
      nextImageAt_(0),
      imageStartedAt_(0),
      lastDataAt_(0),
      imageByteCount_(0),
      previousByte_(0),
      jpegStarted_(false),
      currentFilename_{} {}

void CameraCaptureManager::begin() {
  pinMode(cameraChipSelectPin_, OUTPUT);
  pinMode(sdChipSelectPin_, OUTPUT);
  deselectCamera();
  deselectSd();

  cameraReady_ = camera_.begin() == CAM_ERR_SUCCESS;
  deselectCamera();
  deselectSd();
}

bool CameraCaptureManager::requestBurst(uint8_t photoCount,
                                        unsigned long intervalMs,
                                        unsigned long now) {
  if (!cameraReady_ || !imageStore_.isReady() || isBusy() || photoCount == 0) {
    return false;
  }

  photosRemaining_ = photoCount;
  intervalMs_ = intervalMs;
  nextImageAt_ = now;
  state_ = State::kWaitingForNextImage;
  return true;
}

bool CameraCaptureManager::requestSafeEject() {
  if (sdSafeToRemove_) {
    return true;
  }

  if (!imageStore_.isReady()) {
    return false;
  }

  if (state_ == State::kWaitingForNextImage) {
    photosRemaining_ = 0;
    state_ = State::kIdle;
    finishSafeEject();
    return true;
  }

  if (isBusy()) {
    safeEjectPending_ = true;
    Serial.println(F("SD eject will complete after the current image."));
    return true;
  }

  finishSafeEject();
  return sdSafeToRemove_;
}

void CameraCaptureManager::update(unsigned long now) {
  if (state_ == State::kWaitingForNextImage &&
      static_cast<long>(now - nextImageAt_) >= 0) {
    startNextImage(now);
  }

  if (state_ == State::kReadingImage) {
    processImageBytes(now);
  }
}

bool CameraCaptureManager::isBusy() const { return state_ != State::kIdle; }

bool CameraCaptureManager::isReady() const {
  return cameraReady_ && imageStore_.isReady();
}

bool CameraCaptureManager::isSdSafeToRemove() const { return sdSafeToRemove_; }

void CameraCaptureManager::startNextImage(unsigned long now) {
  if (photosRemaining_ == 0) {
    state_ = State::kIdle;
    return;
  }

  deselectSd();
  if (camera_.takePicture(CAM_IMAGE_MODE_QVGA, CAM_IMAGE_PIX_FMT_JPG) !=
      CAM_ERR_SUCCESS) {
    abortBurst(F("camera did not accept capture request"));
    return;
  }
  deselectCamera();
  imageStartedAt_ = now;
  lastDataAt_ = now;
  imageByteCount_ = 0;
  previousByte_ = 0;
  jpegStarted_ = false;
  currentFilename_[0] = '\0';
  state_ = State::kReadingImage;
}

void CameraCaptureManager::processImageBytes(unsigned long now) {
  uint8_t processed = 0;
  while (camera_.getReceivedLength() > 0 && processed < kBytesPerUpdate) {
    deselectSd();
    const uint8_t currentByte = camera_.readByte();
    deselectCamera();
    ++processed;
    lastDataAt_ = now;

    if (!jpegStarted_) {
      if (previousByte_ == 0xFF && currentByte == 0xD8) {
        currentFile_ = imageStore_.openNextImage(currentFilename_,
                                                 sizeof(currentFilename_));
        if (!currentFile_) {
          abortBurst(F("could not create image file"));
          return;
        }
        jpegStarted_ = true;
        const uint8_t header[] = {0xFF, 0xD8};
        deselectCamera();
        if (currentFile_.write(header, sizeof(header)) != sizeof(header)) {
          abortBurst(F("SD write failed"));
          return;
        }
        deselectSd();
        imageByteCount_ = sizeof(header);
      }
    } else {
      deselectCamera();
      if (currentFile_.write(currentByte) != 1) {
        abortBurst(F("SD write failed"));
        return;
      }
      deselectSd();
      ++imageByteCount_;

      if (previousByte_ == 0xFF && currentByte == 0xD9) {
        finishImage(now);
        return;
      }
    }

    previousByte_ = currentByte;
  }

  if (static_cast<unsigned long>(now - lastDataAt_) > kImageReceiveTimeoutMs ||
      static_cast<unsigned long>(now - imageStartedAt_) >
          kImageReceiveTimeoutMs) {
    abortBurst(F("camera image receive timeout"));
  }
}

void CameraCaptureManager::finishImage(unsigned long now) {
  closeCurrentFile();
  Serial.print(F("Saved "));
  Serial.print(currentFilename_);
  Serial.print(F(" ("));
  Serial.print(imageByteCount_);
  Serial.println(F(" bytes)."));

  --photosRemaining_;
  if (safeEjectPending_) {
    photosRemaining_ = 0;
    state_ = State::kIdle;
    finishSafeEject();
    return;
  }

  if (photosRemaining_ == 0) {
    state_ = State::kIdle;
    Serial.println(F("Camera burst complete."));
    return;
  }

  nextImageAt_ = now + intervalMs_;
  state_ = State::kWaitingForNextImage;
}

void CameraCaptureManager::finishSafeEject() {
  closeCurrentFile();
  deselectCamera();
  imageStore_.end();
  deselectSd();
  safeEjectPending_ = false;
  sdSafeToRemove_ = true;
  Serial.println(F("SD card is safe to remove."));
}

void CameraCaptureManager::abortBurst(const __FlashStringHelper* reason) {
  closeCurrentFile();
  deselectCamera();
  deselectSd();
  photosRemaining_ = 0;
  state_ = State::kIdle;
  Serial.print(F("Camera burst aborted: "));
  Serial.println(reason);
  if (safeEjectPending_) {
    finishSafeEject();
  }
}

void CameraCaptureManager::closeCurrentFile() {
  if (currentFile_) {
    currentFile_.close();
  }
}

void CameraCaptureManager::deselectCamera() const {
  digitalWrite(cameraChipSelectPin_, HIGH);
}

void CameraCaptureManager::deselectSd() const {
  digitalWrite(sdChipSelectPin_, HIGH);
}
