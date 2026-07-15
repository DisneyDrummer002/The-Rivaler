#pragma once

#include <Arduino.h>
#include <Servo.h>

class LauncherController {
 public:
  void begin();
  void update(bool armSwitchHeld, bool safetyStopActive, unsigned long nowMs);
  bool requestQuickShoot(unsigned long nowMs);
  bool requestRapidFire(unsigned long nowMs);

  bool isArmed() const;
  bool isArming() const;
  bool isQuickShootActive() const;
  void disarm();

 private:
  enum class State : uint8_t {
    kDisarmed,
    kArming,
    kArmed,
  };

  enum class MagazineState : uint8_t {
    kAtRest,
    kMovingToShotPosition,
    kWaitingBetweenRapidShots,
  };

  void startMotor();
  void stopMotor();
  void moveToNextShotPosition(unsigned long nowMs);
  void writeMagazineAngle(int16_t angleDegrees);
  void stopMagazine();
  void updateMagazine(unsigned long nowMs);

  State state_ = State::kDisarmed;
  MagazineState magazineState_ = MagazineState::kAtRest;
  bool rapidFireActive_ = false;
  uint8_t rapidShotsCompleted_ = 0;
  uint8_t nextShotPositionIndex_ = 0;
  int16_t magazineAngleDegrees_ = 0;
  unsigned long armingStartedMs_ = 0;
  unsigned long magazineMotionStartedMs_ = 0;
  Servo magazineServo_;
};
