#pragma once

#include <Arduino.h>

#include <RivalerProtocol.h>

class RemoteInputs {
 public:
  void begin();
  void update(unsigned long nowMs);

  rivaler::RemoteControlPayload createControlPayload() const;
  void markEventsQueued(uint16_t queuedEvents);

 private:
  struct InputButton {
    int pin;
    uint16_t heldBit;
    uint16_t eventBit;
    bool stableActive;
    bool rawActive;
    unsigned long rawChangedAtMs;
  };

  static bool isInputActive(int pin);
  static int8_t scaleJoystickAxis(int rawValue, int centerValue);
  void initializeButton(InputButton& button, int pin, uint16_t heldBit,
                        uint16_t eventBit, unsigned long nowMs);
  void updateButton(InputButton& button, unsigned long nowMs);

  InputButton quickShootButton_{};
  InputButton rapidFireButton_{};
  InputButton takePictureButton_{};
  InputButton ejectSdButton_{};
  InputButton followLineButton_{};
  InputButton armSwitch_{};
  uint16_t pendingEvents_ = 0;
};
