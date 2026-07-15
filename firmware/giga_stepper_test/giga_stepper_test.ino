// One-off NEMA 17 / KS0448 L298P shield test for the Arduino GIGA.
//
// Left stepper wiring:
//   One complete coil pair -> shield channel A
//   Other complete coil pair -> shield channel B
//
// Right stepper wiring:
//   One complete coil pair -> shield channel C
//   Other complete coil pair -> shield channel D

constexpr uint8_t kMotorADirectionPin = 3;
constexpr uint8_t kMotorAPwmPin = 6;
constexpr uint8_t kMotorBDirectionPin = 4;
constexpr uint8_t kMotorBPwmPin = 5;
constexpr uint8_t kMotorCDirectionPin = 7;
constexpr uint8_t kMotorCPwmPin = 10;
constexpr uint8_t kMotorDDirectionPin = 8;
constexpr uint8_t kMotorDPwmPin = 9;

// Match the proven dispenser sketch: one full step every 2 ms.
constexpr unsigned long kStepIntervalMs = 2;
constexpr unsigned long kRunTimeMs = 2000;
constexpr unsigned long kPauseTimeMs = 750;

constexpr bool kFullStepPhases[4][2] = {
    {false, true},
    {false, false},
    {true, false},
    {true, true},
};

struct Stepper {
  uint8_t coil1DirectionPin;
  uint8_t coil1PwmPin;
  uint8_t coil2DirectionPin;
  uint8_t coil2PwmPin;
  uint8_t phaseIndex;
};

Stepper leftStepper = {
    kMotorADirectionPin, kMotorAPwmPin,
    kMotorBDirectionPin, kMotorBPwmPin,
    0,
};

Stepper rightStepper = {
    kMotorCDirectionPin, kMotorCPwmPin,
    kMotorDDirectionPin, kMotorDPwmPin,
    0,
};

void applyPhase(Stepper& motor) {
  digitalWrite(motor.coil1DirectionPin,
               kFullStepPhases[motor.phaseIndex][0]);
  digitalWrite(motor.coil2DirectionPin,
               kFullStepPhases[motor.phaseIndex][1]);
}

void runMotor(Stepper& motor, bool forward, unsigned long durationMs) {
  const unsigned long stepCount = durationMs / kStepIntervalMs;

  for (unsigned long step = 0; step < stepCount; ++step) {
    motor.phaseIndex =
        (motor.phaseIndex + (forward ? 1 : 3)) % 4;
    applyPhase(motor);
    delay(kStepIntervalMs);
  }
}

void configureMotorPins(const Stepper& motor) {
  pinMode(motor.coil1DirectionPin, OUTPUT);
  pinMode(motor.coil1PwmPin, OUTPUT);
  pinMode(motor.coil2DirectionPin, OUTPUT);
  pinMode(motor.coil2PwmPin, OUTPUT);
}

void setup() {
  Serial.begin(115200);

  configureMotorPins(leftStepper);
  configureMotorPins(rightStepper);

  // Keep all four channels at full power without using PWM timers.
  digitalWrite(kMotorAPwmPin, HIGH);
  digitalWrite(kMotorBPwmPin, HIGH);
  digitalWrite(kMotorCPwmPin, HIGH);
  digitalWrite(kMotorDPwmPin, HIGH);

  applyPhase(leftStepper);
  applyPhase(rightStepper);

  delay(1000);
}

void loop() {
  Serial.println("Left motor: forward");
  runMotor(leftStepper, true, kRunTimeMs);
  delay(kPauseTimeMs);

  Serial.println("Left motor: backward");
  runMotor(leftStepper, false, kRunTimeMs);
  delay(kPauseTimeMs);

  Serial.println("Right motor: forward");
  runMotor(rightStepper, true, kRunTimeMs);
  delay(kPauseTimeMs);

  Serial.println("Right motor: backward");
  runMotor(rightStepper, false, kRunTimeMs);
  delay(2000);
}
