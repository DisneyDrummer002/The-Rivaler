#include "RemoteInputs.h"

#include "Config.h"

using namespace rivaler_remote_config;

void RemoteInputs::begin() {
  const unsigned long nowMs = millis();

  initializeButton(quickShootButton_, kQuickShootButtonPin, 0,
                   rivaler::kQuickShootPressed, nowMs);
  initializeButton(rapidFireButton_, kRapidFireButtonPin, 0,
                   rivaler::kRapidFirePressed, nowMs);
  initializeButton(takePictureButton_, kTakePictureButtonPin, 0,
                   rivaler::kTakePicturePressed, nowMs);
  initializeButton(ejectSdButton_, kEjectSdButtonPin, 0,
                   rivaler::kEjectSdPressed, nowMs);
  initializeButton(followLineButton_, kFollowLineButtonPin,
                   rivaler::kFollowLineHeld, rivaler::kFollowLinePressed,
                   nowMs);
  initializeButton(armSwitch_, kArmSwitchPin, rivaler::kArmSwitchHeld, 0,
                   nowMs);
  initializeButton(emergencyStopButton_, kEmergencyStopButtonPin,
                   rivaler::kEmergencyStopHeld, 0, nowMs);
}

void RemoteInputs::update(unsigned long nowMs) {
  updateButton(quickShootButton_, nowMs);
  updateButton(rapidFireButton_, nowMs);
  updateButton(takePictureButton_, nowMs);
  updateButton(ejectSdButton_, nowMs);
  updateButton(followLineButton_, nowMs);
  updateButton(armSwitch_, nowMs);
  updateButton(emergencyStopButton_, nowMs);
}

rivaler::RemoteControlPayload RemoteInputs::createControlPayload(
    uint16_t edgeEvents) const {
  rivaler::RemoteControlPayload payload{};
  payload.joystickX = scaleJoystickAxis(analogRead(kJoystickXPin),
                                        kJoystickXCenter);
  payload.joystickY = scaleJoystickAxis(analogRead(kJoystickYPin),
                                        kJoystickYCenter);
  if (!kJoystickYPositiveIsForward) {
    payload.joystickY = -payload.joystickY;
  }

  const InputButton* buttons[] = {
      &quickShootButton_, &rapidFireButton_, &takePictureButton_,
      &ejectSdButton_,    &followLineButton_, &armSwitch_,
      &emergencyStopButton_,
  };
  for (const InputButton* button : buttons) {
    if (button->stableActive) {
      payload.heldButtons |= button->heldBit;
    }
  }

  payload.edgeEvents = edgeEvents;
  return payload;
}

uint16_t RemoteInputs::pendingEvents() const { return pendingEvents_; }

void RemoteInputs::acknowledgeEvents(uint16_t acknowledgedEvents) {
  pendingEvents_ &= ~acknowledgedEvents;
}

bool RemoteInputs::isInputActive(int pin) {
  const bool pinIsLow = digitalRead(pin) == LOW;
  return kInputsAreActiveLow ? pinIsLow : !pinIsLow;
}

int8_t RemoteInputs::scaleJoystickAxis(int rawValue, int centerValue) {
  const int delta = rawValue - centerValue;
  const int magnitude = abs(delta);
  if (magnitude <= kJoystickDeadZone) {
    return 0;
  }

  const int availableRange =
      delta > 0 ? kJoystickAdcMaximum - centerValue : centerValue;
  if (availableRange <= kJoystickDeadZone) {
    return 0;
  }

  int scaledMagnitude =
      ((magnitude - kJoystickDeadZone) * 100) /
      (availableRange - kJoystickDeadZone);
  scaledMagnitude = constrain(scaledMagnitude, 0, 100);
  return static_cast<int8_t>(delta > 0 ? scaledMagnitude : -scaledMagnitude);
}

void RemoteInputs::initializeButton(InputButton& button, int pin,
                                    uint16_t heldBit, uint16_t eventBit,
                                    unsigned long nowMs) {
  button.pin = pin;
  button.heldBit = heldBit;
  button.eventBit = eventBit;
  pinMode(pin, INPUT_PULLUP);
  button.rawActive = isInputActive(pin);
  button.stableActive = button.rawActive;
  button.rawChangedAtMs = nowMs;
}

void RemoteInputs::updateButton(InputButton& button, unsigned long nowMs) {
  const bool rawActive = isInputActive(button.pin);
  if (rawActive != button.rawActive) {
    button.rawActive = rawActive;
    button.rawChangedAtMs = nowMs;
  }

  if (button.stableActive == button.rawActive ||
      nowMs - button.rawChangedAtMs < kDebounceMs) {
    return;
  }

  button.stableActive = button.rawActive;
  if (button.stableActive && button.eventBit != 0) {
    pendingEvents_ |= button.eventBit;
  }
}
