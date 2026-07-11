#include "CameraCaptureManager.h"
#include "Config.h"
#include "MegaCommandReceiver.h"
#include "SdImageStore.h"

namespace {

MegaCommandReceiver commandReceiver(Serial1);
SdImageStore imageStore(rivaler_mega_config::kCameraChipSelectPin,
                        rivaler_mega_config::kSdChipSelectPin);
CameraCaptureManager camera(rivaler_mega_config::kCameraChipSelectPin,
                            rivaler_mega_config::kSdChipSelectPin, imageStore);

void logCommand(const rivaler::CameraCommandPayload& command) {
  if (command.operation == rivaler::CameraOperation::kCaptureBurst) {
    Serial.print("Camera capture requested: ");
    Serial.print(command.photoCount);
    Serial.print(" photo(s), ");
    Serial.print(command.intervalMs);
    Serial.println(" ms interval.");
    return;
  }

  Serial.println("SD safe-eject requested.");
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("Rivaler Mega camera controller starting.");

  // Serial1 RX is Mega D19, wired from the Robot Nano. Mega TX is unused.
  commandReceiver.begin(rivaler_mega_config::kRobotNanoCommandBaud);

  if (imageStore.begin()) {
    Serial.println("SD card ready.");
  } else {
    Serial.println("SD card initialization failed; capture requests are rejected.");
  }

  camera.begin();
  if (camera.isReady()) {
    Serial.println("Arducam B0401 capture manager ready.");
  } else {
    Serial.println("Arducam initialization failed; capture requests are rejected.");
  }
}

void loop() {
  rivaler::CameraCommandPayload command{};
  while (commandReceiver.readCommand(command)) {
    logCommand(command);

    if (command.operation == rivaler::CameraOperation::kCaptureBurst &&
        !camera.requestBurst(command.photoCount, command.intervalMs, millis())) {
      Serial.println("Camera capture request rejected.");
    }

    if (command.operation == rivaler::CameraOperation::kPrepareSdEject &&
        !camera.requestSafeEject()) {
      Serial.println("SD safe-eject request rejected.");
    }
  }

  camera.update(millis());
}
