#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <math.h>

// Use the CH343 USB-UART bridge on COM6, not native ESP32-S3 USB CDC.
#define Serial Serial0

#ifndef STEPPER_DIR_PIN
#define STEPPER_DIR_PIN 16
#endif

#ifndef STEPPER_STEP_PIN
#define STEPPER_STEP_PIN 17
#endif

#ifndef STEPPER_M0_PIN
#define STEPPER_M0_PIN 3
#endif

#ifndef STEPPER_M1_PIN
#define STEPPER_M1_PIN 8
#endif

#ifndef STEPPER_M2_PIN
#define STEPPER_M2_PIN 18
#endif

#ifndef STEPPER_ENABLE_PIN
#define STEPPER_ENABLE_PIN -1
#endif

#ifndef SERVO_PIN
#define SERVO_PIN 47
#endif

#ifndef ESP_RGB_LED_PIN
#define ESP_RGB_LED_PIN 48
#endif

namespace {
constexpr float kMotorFullStepsPerRevolution = 200.0f;
constexpr float kDefaultGearRatio = 1.0f;
constexpr uint32_t kStepPulseHighUs = 20;
constexpr uint32_t kDirectionSetupUs = 5;
constexpr uint32_t kStatusIntervalMs = 1000;
constexpr uint32_t kDefaultRampMs = 1000;
constexpr float kDefaultRpm = 60.0f;
constexpr int kDefaultMicrosteps = 8;
constexpr unsigned long kSerialWaitMs = 3000;

constexpr uint8_t kServoPwmResolutionBits = 16;
constexpr uint32_t kServoPwmMaxDuty = (1UL << kServoPwmResolutionBits) - 1;
constexpr int kDefaultServoMinUs = 500;
constexpr int kDefaultServoMaxUs = 2500;
constexpr int kDefaultServoFrequencyHz = 50;
constexpr float kDefaultServoRangeDegrees = 270.0f;
constexpr uint32_t kDefaultServoMoveDurationMs = 300;
constexpr uint32_t kServoUpdateIntervalMs = 20;

enum class MotionMode {
  Stopped,
  Continuous,
  Move,
};

MotionMode motionMode = MotionMode::Stopped;
bool directionForward = true;
int microsteps = kDefaultMicrosteps;
float gearRatio = kDefaultGearRatio;
float targetRpm = kDefaultRpm;
uint32_t rampDurationMs = kDefaultRampMs;
uint32_t rampStartMs = 0;
float rampStartIntervalUs = 0.0f;
float targetIntervalUs = 0.0f;
float currentIntervalUs = 0.0f;
uint32_t lastStepUs = 0;
uint32_t totalSteps = 0;
uint32_t moveStepsRemaining = 0;

int stepperDirPin = STEPPER_DIR_PIN;
int stepperStepPin = STEPPER_STEP_PIN;
int stepperM0Pin = STEPPER_M0_PIN;
int stepperM1Pin = STEPPER_M1_PIN;
int stepperM2Pin = STEPPER_M2_PIN;
int stepperEnablePin = STEPPER_ENABLE_PIN;

int servoPin = SERVO_PIN;
int servoMinUs = kDefaultServoMinUs;
int servoMaxUs = kDefaultServoMaxUs;
int servoFrequencyHz = kDefaultServoFrequencyHz;
float servoRangeDegrees = kDefaultServoRangeDegrees;
float servoAngleDegrees = kDefaultServoRangeDegrees / 2.0f;
float servoTargetAngleDegrees = kDefaultServoRangeDegrees / 2.0f;
float servoMoveStartAngleDegrees = kDefaultServoRangeDegrees / 2.0f;
float servoMoveTargetAngleDegrees = kDefaultServoRangeDegrees / 2.0f;
int servoPulseUs = 1500;
uint32_t servoDuty = 0;
bool servoAttached = false;
bool servoSweepEnabled = false;
bool servoMoveActive = false;
float servoSweepMinDegrees = 0.0f;
float servoSweepMaxDegrees = kDefaultServoRangeDegrees;
uint32_t servoMoveDurationMs = kDefaultServoMoveDurationMs;
uint32_t servoMoveStartMs = 0;
uint32_t servoSweepPeriodMs = 2000;
uint32_t servoSweepStartMs = 0;
uint32_t lastServoWriteMs = 0;

esp_timer_handle_t servoFrameTimer = nullptr;
esp_timer_handle_t servoPulseEndTimer = nullptr;
portMUX_TYPE servoTimerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool timerServoAttached = false;
volatile int timerServoPin = SERVO_PIN;
volatile int timerServoPulseUs = 1500;

uint32_t lastStatusMs = 0;
String commandLine;

void servoPulseEndCallback(void*) {
  int pin = SERVO_PIN;
  bool attached = false;
  portENTER_CRITICAL(&servoTimerMux);
  attached = timerServoAttached;
  pin = timerServoPin;
  portEXIT_CRITICAL(&servoTimerMux);

  if (attached) {
    gpio_set_level(static_cast<gpio_num_t>(pin), 0);
  }
}

void servoFrameCallback(void*) {
  int pin = SERVO_PIN;
  int pulseUs = 1500;
  bool attached = false;
  portENTER_CRITICAL(&servoTimerMux);
  attached = timerServoAttached;
  pin = timerServoPin;
  pulseUs = timerServoPulseUs;
  portEXIT_CRITICAL(&servoTimerMux);

  if (!attached || pulseUs <= 0) {
    return;
  }

  gpio_set_level(static_cast<gpio_num_t>(pin), 1);
  esp_timer_stop(servoPulseEndTimer);
  esp_timer_start_once(servoPulseEndTimer, pulseUs);
}

void turnOffOnboardRgbLed() {
#if defined(ESP32)
  pinMode(ESP_RGB_LED_PIN, OUTPUT);
  digitalWrite(ESP_RGB_LED_PIN, LOW);
  neopixelWrite(ESP_RGB_LED_PIN, 0, 0, 0);
#endif
}

const char* stepperModeName() {
  switch (motionMode) {
    case MotionMode::Continuous:
      return "run";
    case MotionMode::Move:
      return "move";
    case MotionMode::Stopped:
    default:
      return "stop";
  }
}

float stepsPerRevolution() {
  return kMotorFullStepsPerRevolution * static_cast<float>(microsteps) *
         gearRatio;
}

float intervalFromRpm(float rpm) {
  const float safeRpm = constrain(rpm, 0.1f, 600.0f);
  const float stepsPerSecond = (safeRpm * stepsPerRevolution()) / 60.0f;
  return 1000000.0f / stepsPerSecond;
}

void applyMicrostepPins(int requestedMicrosteps) {
  microsteps = requestedMicrosteps;

  bool m0 = LOW;
  bool m1 = LOW;
  bool m2 = LOW;

  switch (microsteps) {
    case 1:
      m0 = LOW;
      m1 = LOW;
      m2 = LOW;
      break;
    case 2:
      m0 = HIGH;
      m1 = LOW;
      m2 = LOW;
      break;
    case 4:
      m0 = LOW;
      m1 = HIGH;
      m2 = LOW;
      break;
    case 8:
      m0 = HIGH;
      m1 = HIGH;
      m2 = LOW;
      break;
    case 16:
      m0 = LOW;
      m1 = LOW;
      m2 = HIGH;
      break;
    case 32:
    default:
      microsteps = 32;
      m0 = HIGH;
      m1 = HIGH;
      m2 = HIGH;
      break;
  }

  digitalWrite(stepperM0Pin, m0);
  digitalWrite(stepperM1Pin, m1);
  digitalWrite(stepperM2Pin, m2);
}

void configureStepperPins() {
  pinMode(stepperDirPin, OUTPUT);
  pinMode(stepperStepPin, OUTPUT);
  pinMode(stepperM0Pin, OUTPUT);
  pinMode(stepperM1Pin, OUTPUT);
  pinMode(stepperM2Pin, OUTPUT);

  digitalWrite(stepperDirPin, directionForward ? HIGH : LOW);
  digitalWrite(stepperStepPin, LOW);
  applyMicrostepPins(microsteps);

  if (stepperEnablePin >= 0) {
    pinMode(stepperEnablePin, OUTPUT);
    digitalWrite(stepperEnablePin, LOW);
  }
}

void applyStepperDirection(bool forward) {
  directionForward = forward;
  digitalWrite(stepperDirPin, directionForward ? HIGH : LOW);
  delayMicroseconds(kDirectionSetupUs);
}

void setStepperPins(int dirPin, int stepPin, int m0Pin, int m1Pin, int m2Pin,
                    int enablePin) {
  motionMode = MotionMode::Stopped;
  stepperDirPin = dirPin;
  stepperStepPin = stepPin;
  stepperM0Pin = m0Pin;
  stepperM1Pin = m1Pin;
  stepperM2Pin = m2Pin;
  stepperEnablePin = enablePin;
  configureStepperPins();
  Serial.printf("STEPPERPINS dir=%d step=%d m0=%d m1=%d m2=%d enable=%d\r\n",
                stepperDirPin, stepperStepPin, stepperM0Pin, stepperM1Pin,
                stepperM2Pin, stepperEnablePin);
}

void updateRampTarget(float rpm, uint32_t rampMs) {
  targetRpm = constrain(rpm, 0.1f, 600.0f);
  rampDurationMs = max<uint32_t>(rampMs, 1);
  rampStartMs = millis();
  rampStartIntervalUs =
      currentIntervalUs > 0.0f ? currentIntervalUs : intervalFromRpm(5.0f);
  targetIntervalUs = intervalFromRpm(targetRpm);
}

void setGearRatio(float requestedGearRatio) {
  gearRatio = constrain(requestedGearRatio, 0.01f, 500.0f);
  updateRampTarget(targetRpm, rampDurationMs);
  Serial.printf("GEAR ratio=%.4f\r\n", gearRatio);
}

void stopStepper(const char* reason) {
  motionMode = MotionMode::Stopped;
  moveStepsRemaining = 0;
  Serial.printf("STEPPER_STOPPED %s\r\n", reason);
}

void startContinuous(bool forward, float rpm, int requestedMicrosteps,
                     uint32_t rampMs, float requestedGearRatio) {
  applyStepperDirection(forward);
  applyMicrostepPins(requestedMicrosteps);
  gearRatio = constrain(requestedGearRatio, 0.01f, 500.0f);
  updateRampTarget(rpm, rampMs);
  lastStepUs = micros();
  motionMode = MotionMode::Continuous;
  Serial.printf("RUN %s rpm=%.2f microsteps=%d ramp_ms=%lu gear=%.4f\r\n",
                directionForward ? "FWD" : "REV", targetRpm, microsteps,
                static_cast<unsigned long>(rampDurationMs), gearRatio);
}

void startMove(bool forward, float degrees, float rpm, int requestedMicrosteps,
               uint32_t rampMs, float requestedGearRatio) {
  applyStepperDirection(forward);
  applyMicrostepPins(requestedMicrosteps);
  gearRatio = constrain(requestedGearRatio, 0.01f, 500.0f);
  updateRampTarget(rpm, rampMs);
  moveStepsRemaining = static_cast<uint32_t>(
      lroundf((fabsf(degrees) / 360.0f) * stepsPerRevolution()));
  lastStepUs = micros();
  motionMode = moveStepsRemaining > 0 ? MotionMode::Move : MotionMode::Stopped;
  Serial.printf("MOVE %s degrees=%.2f steps=%lu rpm=%.2f microsteps=%d gear=%.4f\r\n",
                directionForward ? "FWD" : "REV", degrees,
                static_cast<unsigned long>(moveStepsRemaining), targetRpm,
                microsteps, gearRatio);
}

float rampedIntervalUs() {
  if (motionMode == MotionMode::Stopped) {
    return targetIntervalUs;
  }

  const float progress =
      constrain(static_cast<float>(millis() - rampStartMs) / rampDurationMs,
                0.0f, 1.0f);
  const float smoothProgress = progress * progress * (3.0f - 2.0f * progress);
  currentIntervalUs =
      rampStartIntervalUs +
      ((targetIntervalUs - rampStartIntervalUs) * smoothProgress);

  return currentIntervalUs;
}

void pulseStepPin() {
  digitalWrite(stepperStepPin, HIGH);
  delayMicroseconds(kStepPulseHighUs);
  digitalWrite(stepperStepPin, LOW);
}

void maybeStep(uint32_t nowUs) {
  if (motionMode == MotionMode::Stopped) {
    return;
  }

  const float intervalUs = rampedIntervalUs();
  if (static_cast<float>(nowUs - lastStepUs) < intervalUs) {
    return;
  }

  digitalWrite(stepperDirPin, directionForward ? HIGH : LOW);
  delayMicroseconds(kDirectionSetupUs);
  pulseStepPin();

  lastStepUs = micros();
  totalSteps++;

  if (motionMode == MotionMode::Move) {
    if (moveStepsRemaining > 0) {
      moveStepsRemaining--;
    }

    if (moveStepsRemaining == 0) {
      stopStepper("move_complete");
    }
  }
}

uint32_t servoDutyFromPulseUs(int pulseUs) {
  const float periodUs = 1000000.0f / servoFrequencyHz;
  const float duty = (constrain(pulseUs, servoMinUs, servoMaxUs) / periodUs) *
                     kServoPwmMaxDuty;
  return static_cast<uint32_t>(constrain(duty, 0.0f,
                                        static_cast<float>(kServoPwmMaxDuty)));
}

void ensureServoTimers() {
  if (servoPulseEndTimer == nullptr) {
    esp_timer_create_args_t pulseEndArgs = {};
    pulseEndArgs.callback = &servoPulseEndCallback;
    pulseEndArgs.name = "servo_low";
    esp_timer_create(&pulseEndArgs, &servoPulseEndTimer);
  }

  if (servoFrameTimer == nullptr) {
    esp_timer_create_args_t frameArgs = {};
    frameArgs.callback = &servoFrameCallback;
    frameArgs.name = "servo_frame";
    esp_timer_create(&frameArgs, &servoFrameTimer);
  }
}

void syncServoTimerState() {
  portENTER_CRITICAL(&servoTimerMux);
  timerServoAttached = servoAttached;
  timerServoPin = servoPin;
  timerServoPulseUs = servoPulseUs;
  portEXIT_CRITICAL(&servoTimerMux);
}

void stopServoTimers() {
  portENTER_CRITICAL(&servoTimerMux);
  timerServoAttached = false;
  portEXIT_CRITICAL(&servoTimerMux);

  if (servoFrameTimer != nullptr) {
    esp_timer_stop(servoFrameTimer);
  }
  if (servoPulseEndTimer != nullptr) {
    esp_timer_stop(servoPulseEndTimer);
  }
}

void startServoTimers() {
  ensureServoTimers();
  stopServoTimers();
  syncServoTimerState();

  const uint64_t frameIntervalUs = 1000000ULL / servoFrequencyHz;
  esp_timer_start_periodic(servoFrameTimer, frameIntervalUs);
}

int servoPulseFromAngle(float angleDegrees) {
  const float constrainedAngle = constrain(angleDegrees, 0.0f, servoRangeDegrees);
  return static_cast<int>(lroundf(
      servoMinUs +
      ((servoMaxUs - servoMinUs) * (constrainedAngle / servoRangeDegrees))));
}

float servoAngleFromPulse(int pulseUs) {
  if (servoMaxUs <= servoMinUs) {
    return 0.0f;
  }

  return constrain(((pulseUs - servoMinUs) * servoRangeDegrees) /
                       static_cast<float>(servoMaxUs - servoMinUs),
                   0.0f, servoRangeDegrees);
}

void applyServoPulse(int pulseUs) {
  servoPulseUs = constrain(pulseUs, servoMinUs, servoMaxUs);
  servoAngleDegrees = servoAngleFromPulse(servoPulseUs);
  servoDuty = servoDutyFromPulseUs(servoPulseUs);
  if (servoAttached) {
    syncServoTimerState();
  }
}

void servoWritePulse(int pulseUs) {
  servoMoveActive = false;
  applyServoPulse(pulseUs);
  servoTargetAngleDegrees = servoAngleDegrees;
}

void attachServo(int pin, int minUs, int maxUs, int frequencyHz) {
  if (servoAttached) {
    stopServoTimers();
    digitalWrite(servoPin, LOW);
  }

  servoPin = pin;
  servoMinUs = constrain(minUs, 100, 3000);
  servoMaxUs = constrain(maxUs, servoMinUs + 1, 3500);
  servoFrequencyHz = constrain(frequencyHz, 20, 400);
  servoSweepEnabled = false;
  servoMoveActive = false;

  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);
  servoAttached = true;
  servoWritePulse(servoPulseUs);
  startServoTimers();

  Serial.printf("SERVO attached pin=%d min_us=%d max_us=%d freq=%d range=%.2f\r\n",
                servoPin, servoMinUs, servoMaxUs, servoFrequencyHz,
                servoRangeDegrees);
}

void detachServo() {
  servoSweepEnabled = false;
  servoMoveActive = false;
  stopServoTimers();
  if (servoAttached) {
    digitalWrite(servoPin, LOW);
  }
  servoAttached = false;
  syncServoTimerState();
  Serial.println("SERVO detached");
}

void servoWriteAngle(float angleDegrees, uint32_t moveMs) {
  servoSweepEnabled = false;
  servoTargetAngleDegrees = constrain(angleDegrees, 0.0f, servoRangeDegrees);

  if (moveMs <= kServoUpdateIntervalMs) {
    servoMoveActive = false;
    servoAngleDegrees = servoTargetAngleDegrees;
    applyServoPulse(servoPulseFromAngle(servoAngleDegrees));
  } else {
    servoMoveStartAngleDegrees = servoAngleDegrees;
    servoMoveTargetAngleDegrees = servoTargetAngleDegrees;
    servoMoveDurationMs = constrain(moveMs, 20UL, 10000UL);
    servoMoveStartMs = millis();
    servoMoveActive = true;
  }

  Serial.printf("SERVO target_angle=%.2f move_ms=%lu pulse_us=%d\r\n",
                servoTargetAngleDegrees,
                static_cast<unsigned long>(moveMs), servoPulseUs);
}

void setServoRange(float rangeDegrees) {
  servoRangeDegrees = constrain(rangeDegrees, 1.0f, 360.0f);
  servoSweepMinDegrees = constrain(servoSweepMinDegrees, 0.0f, servoRangeDegrees);
  servoSweepMaxDegrees = constrain(servoSweepMaxDegrees, 0.0f, servoRangeDegrees);
  servoAngleDegrees = constrain(servoAngleDegrees, 0.0f, servoRangeDegrees);
  servoTargetAngleDegrees =
      constrain(servoTargetAngleDegrees, 0.0f, servoRangeDegrees);
  servoMoveTargetAngleDegrees =
      constrain(servoMoveTargetAngleDegrees, 0.0f, servoRangeDegrees);
  applyServoPulse(servoPulseFromAngle(servoAngleDegrees));
  Serial.printf("SERVO range=%.2f angle=%.2f pulse_us=%d\r\n",
                servoRangeDegrees, servoAngleDegrees, servoPulseUs);
}

void startServoSweep(float minDegrees, float maxDegrees, uint32_t periodMs) {
  servoMoveActive = false;
  servoSweepMinDegrees = constrain(minDegrees, 0.0f, servoRangeDegrees);
  servoSweepMaxDegrees = constrain(maxDegrees, 0.0f, servoRangeDegrees);
  if (servoSweepMaxDegrees < servoSweepMinDegrees) {
    const float tmp = servoSweepMinDegrees;
    servoSweepMinDegrees = servoSweepMaxDegrees;
    servoSweepMaxDegrees = tmp;
  }
  servoSweepPeriodMs = max<uint32_t>(periodMs, 100);
  servoSweepStartMs = millis();
  servoSweepEnabled = true;
  Serial.printf("SERVO sweep min=%.2f max=%.2f period_ms=%lu\r\n",
                servoSweepMinDegrees, servoSweepMaxDegrees,
                static_cast<unsigned long>(servoSweepPeriodMs));
}

void updateServoSweep(uint32_t nowMs) {
  if (!servoAttached || !servoSweepEnabled ||
      nowMs - lastServoWriteMs < kServoUpdateIntervalMs) {
    return;
  }

  lastServoWriteMs = nowMs;
  const uint32_t elapsed = nowMs - servoSweepStartMs;
  const float phase =
      static_cast<float>(elapsed % servoSweepPeriodMs) / servoSweepPeriodMs;
  const float triangle = phase < 0.5f ? phase * 2.0f : (1.0f - phase) * 2.0f;
  const float angle = servoSweepMinDegrees +
                      ((servoSweepMaxDegrees - servoSweepMinDegrees) *
                       triangle);
  servoAngleDegrees = angle;
  servoTargetAngleDegrees = angle;
  applyServoPulse(servoPulseFromAngle(angle));
}

void updateServoMove(uint32_t nowMs) {
  if (!servoAttached || !servoMoveActive ||
      nowMs - lastServoWriteMs < kServoUpdateIntervalMs) {
    return;
  }

  lastServoWriteMs = nowMs;
  const float progress =
      constrain(static_cast<float>(nowMs - servoMoveStartMs) /
                    static_cast<float>(servoMoveDurationMs),
                0.0f, 1.0f);
  const float smoothProgress = progress * progress * (3.0f - 2.0f * progress);
  servoAngleDegrees = servoMoveStartAngleDegrees +
                      ((servoMoveTargetAngleDegrees - servoMoveStartAngleDegrees) *
                       smoothProgress);
  applyServoPulse(servoPulseFromAngle(servoAngleDegrees));

  if (progress >= 1.0f) {
    servoMoveActive = false;
    servoAngleDegrees = servoMoveTargetAngleDegrees;
    applyServoPulse(servoPulseFromAngle(servoAngleDegrees));
  }
}

void printStepperStatus() {
  const float stepsPerSecond =
      currentIntervalUs > 0.0f ? 1000000.0f / currentIntervalUs : 0.0f;
  const float rpm = (stepsPerSecond / stepsPerRevolution()) * 60.0f;

  Serial.printf(
      "STEPPER_STATUS mode=%s dir=%s rpm=%.2f target_rpm=%.2f microsteps=%d "
      "gear_ratio=%.4f steps_per_rev=%.1f interval_us=%.1f "
      "total_steps=%lu remaining=%lu pins=%d,%d,%d,%d,%d,%d\r\n",
      stepperModeName(), directionForward ? "FWD" : "REV", rpm, targetRpm,
      microsteps, gearRatio, stepsPerRevolution(), currentIntervalUs,
      static_cast<unsigned long>(totalSteps),
      static_cast<unsigned long>(moveStepsRemaining), stepperDirPin,
      stepperStepPin, stepperM0Pin, stepperM1Pin, stepperM2Pin,
      stepperEnablePin);
}

void printServoStatus() {
  Serial.printf(
      "SERVO_STATUS attached=%d pin=%d angle=%.2f pulse_us=%d min_us=%d "
      "max_us=%d freq=%d range=%.2f duty=%lu sweep=%d sweep_min=%.2f sweep_max=%.2f "
      "sweep_period_ms=%lu moving=%d target_angle=%.2f move_ms=%lu\r\n",
      servoAttached ? 1 : 0, servoPin, servoAngleDegrees, servoPulseUs,
      servoMinUs, servoMaxUs, servoFrequencyHz, servoRangeDegrees,
      static_cast<unsigned long>(servoDuty), servoSweepEnabled ? 1 : 0,
      servoSweepMinDegrees, servoSweepMaxDegrees,
      static_cast<unsigned long>(servoSweepPeriodMs),
      servoMoveActive ? 1 : 0, servoTargetAngleDegrees,
      static_cast<unsigned long>(servoMoveDurationMs));
}

void printStatus() {
  printStepperStatus();
  printServoStatus();
}

bool parseDirection(const char* text) {
  return strcasecmp(text, "REV") != 0 && strcasecmp(text, "BACK") != 0 &&
         strcasecmp(text, "BACKWARD") != 0;
}

void handleCommand(String rawCommand) {
  rawCommand.trim();
  if (rawCommand.length() == 0) {
    return;
  }

  char buffer[128];
  rawCommand.toCharArray(buffer, sizeof(buffer));

  char command[20] = {};
  sscanf(buffer, "%19s", command);

  if (strcasecmp(command, "STOP") == 0 || strcasecmp(command, "ESTOP") == 0) {
    stopStepper(command);
    return;
  }

  if (strcasecmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcasecmp(command, "STEPPERPINS") == 0) {
    int dirPin = stepperDirPin;
    int stepPin = stepperStepPin;
    int m0Pin = stepperM0Pin;
    int m1Pin = stepperM1Pin;
    int m2Pin = stepperM2Pin;
    int enablePin = stepperEnablePin;
    if (sscanf(buffer, "%*s %d %d %d %d %d %d", &dirPin, &stepPin, &m0Pin,
               &m1Pin, &m2Pin, &enablePin) >= 5) {
      setStepperPins(dirPin, stepPin, m0Pin, m1Pin, m2Pin, enablePin);
    } else {
      Serial.println("ERR usage: STEPPERPINS dir step m0 m1 m2 enable");
    }
    return;
  }

  if (strcasecmp(command, "DIR") == 0) {
    char dirText[16] = {};
    if (sscanf(buffer, "%*s %15s", dirText) == 1) {
      applyStepperDirection(parseDirection(dirText));
      Serial.printf("DIR %s\r\n", directionForward ? "FWD" : "REV");
    }
    return;
  }

  if (strcasecmp(command, "SPEED") == 0) {
    float rpm = targetRpm;
    if (sscanf(buffer, "%*s %f", &rpm) == 1) {
      updateRampTarget(rpm, rampDurationMs);
      Serial.printf("SPEED rpm=%.2f\r\n", targetRpm);
    }
    return;
  }

  if (strcasecmp(command, "RAMP") == 0) {
    uint32_t rampMs = rampDurationMs;
    if (sscanf(buffer, "%*s %lu", &rampMs) == 1) {
      updateRampTarget(targetRpm, rampMs);
      Serial.printf("RAMP ms=%lu\r\n", static_cast<unsigned long>(rampMs));
    }
    return;
  }

  if (strcasecmp(command, "MICROSTEP") == 0) {
    int requestedMicrosteps = microsteps;
    if (sscanf(buffer, "%*s %d", &requestedMicrosteps) == 1) {
      applyMicrostepPins(requestedMicrosteps);
      updateRampTarget(targetRpm, rampDurationMs);
      Serial.printf("MICROSTEP %d\r\n", microsteps);
    }
    return;
  }

  if (strcasecmp(command, "GEAR") == 0) {
    float requestedGearRatio = gearRatio;
    if (sscanf(buffer, "%*s %f", &requestedGearRatio) == 1) {
      setGearRatio(requestedGearRatio);
    } else {
      Serial.println("ERR usage: GEAR ratio");
    }
    return;
  }

  if (strcasecmp(command, "RUN") == 0) {
    char dirText[16] = {};
    float rpm = targetRpm;
    int requestedMicrosteps = microsteps;
    uint32_t rampMs = rampDurationMs;
    float requestedGearRatio = gearRatio;
    const int parsed = sscanf(buffer, "%*s %15s %f %d %lu %f", dirText, &rpm,
                              &requestedMicrosteps, &rampMs,
                              &requestedGearRatio);
    if (parsed >= 2) {
      startContinuous(parseDirection(dirText), rpm, requestedMicrosteps,
                      parsed >= 4 ? rampMs : rampDurationMs,
                      parsed >= 5 ? requestedGearRatio : gearRatio);
    } else {
      Serial.println("ERR usage: RUN FWD|REV rpm microsteps ramp_ms gear_ratio");
    }
    return;
  }

  if (strcasecmp(command, "MOVE") == 0) {
    char dirText[16] = {};
    float degrees = 360.0f;
    float rpm = targetRpm;
    int requestedMicrosteps = microsteps;
    uint32_t rampMs = rampDurationMs;
    float requestedGearRatio = gearRatio;
    const int parsed = sscanf(buffer, "%*s %15s %f %f %d %lu %f", dirText,
                              &degrees, &rpm, &requestedMicrosteps, &rampMs,
                              &requestedGearRatio);
    if (parsed >= 3) {
      startMove(parseDirection(dirText), degrees, rpm, requestedMicrosteps,
                parsed >= 5 ? rampMs : rampDurationMs,
                parsed >= 6 ? requestedGearRatio : gearRatio);
    } else {
      Serial.println("ERR usage: MOVE FWD|REV degrees rpm microsteps ramp_ms gear_ratio");
    }
    return;
  }

  if (strcasecmp(command, "SERVOATTACH") == 0) {
    int pin = servoPin;
    int minUs = servoMinUs;
    int maxUs = servoMaxUs;
    int freq = servoFrequencyHz;
    const int parsed =
        sscanf(buffer, "%*s %d %d %d %d", &pin, &minUs, &maxUs, &freq);
    if (parsed >= 1) {
      attachServo(pin, parsed >= 2 ? minUs : servoMinUs,
                  parsed >= 3 ? maxUs : servoMaxUs,
                  parsed >= 4 ? freq : servoFrequencyHz);
    } else {
      Serial.println("ERR usage: SERVOATTACH pin min_us max_us freq");
    }
    return;
  }

  if (strcasecmp(command, "SERVODETACH") == 0) {
    detachServo();
    return;
  }

  if (strcasecmp(command, "SERVOANGLE") == 0) {
    float angle = servoAngleDegrees;
    uint32_t moveMs = servoMoveDurationMs;
    const int parsed = sscanf(buffer, "%*s %f %lu", &angle, &moveMs);
    if (parsed >= 1) {
      if (!servoAttached) {
        attachServo(servoPin, servoMinUs, servoMaxUs, servoFrequencyHz);
      }
      servoWriteAngle(angle, parsed >= 2 ? moveMs : servoMoveDurationMs);
    } else {
      Serial.println("ERR usage: SERVOANGLE degrees move_ms");
    }
    return;
  }

  if (strcasecmp(command, "SERVOPULSE") == 0) {
    int pulseUs = servoPulseUs;
    if (sscanf(buffer, "%*s %d", &pulseUs) == 1) {
      if (!servoAttached) {
        attachServo(servoPin, servoMinUs, servoMaxUs, servoFrequencyHz);
      }
      servoSweepEnabled = false;
      servoWritePulse(pulseUs);
      Serial.printf("SERVO pulse_us=%d angle=%.2f\r\n", servoPulseUs,
                    servoAngleDegrees);
    } else {
      Serial.println("ERR usage: SERVOPULSE microseconds");
    }
    return;
  }

  if (strcasecmp(command, "SERVORANGE") == 0) {
    float rangeDegrees = servoRangeDegrees;
    if (sscanf(buffer, "%*s %f", &rangeDegrees) == 1) {
      setServoRange(rangeDegrees);
    } else {
      Serial.println("ERR usage: SERVORANGE degrees");
    }
    return;
  }

  if (strcasecmp(command, "SERVOSWEEP") == 0) {
    float minDegrees = servoSweepMinDegrees;
    float maxDegrees = servoSweepMaxDegrees;
    uint32_t periodMs = servoSweepPeriodMs;
    if (sscanf(buffer, "%*s %f %f %lu", &minDegrees, &maxDegrees,
               &periodMs) >= 2) {
      if (!servoAttached) {
        attachServo(servoPin, servoMinUs, servoMaxUs, servoFrequencyHz);
      }
      startServoSweep(minDegrees, maxDegrees, periodMs);
    } else {
      Serial.println("ERR usage: SERVOSWEEP min_deg max_deg period_ms");
    }
    return;
  }

  if (strcasecmp(command, "SERVOSTOP") == 0) {
    servoSweepEnabled = false;
    Serial.println("SERVO sweep_stop");
    return;
  }

  if (strcasecmp(command, "SERVOSTATUS") == 0) {
    printServoStatus();
    return;
  }

  Serial.printf("ERR unknown_command=%s\r\n", command);
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char incoming = static_cast<char>(Serial.read());
    if (incoming == '\n' || incoming == '\r') {
      handleCommand(commandLine);
      commandLine = "";
    } else if (commandLine.length() < 127) {
      commandLine += incoming;
    }
  }
}
}  // namespace

void setup() {
  turnOffOnboardRgbLed();

  Serial.begin(115200);
  const unsigned long startMs = millis();
  while (!Serial && millis() - startMs < kSerialWaitMs) {
    delay(10);
  }

  configureStepperPins();
  currentIntervalUs = intervalFromRpm(5.0f);
  targetIntervalUs = intervalFromRpm(targetRpm);
  lastStepUs = micros();
  lastStatusMs = millis();

  Serial.println();
  Serial.println("ESP32-S3 stepper/servo serial controller ready");
  Serial.println("Stepper commands:");
  Serial.println("STEPPERPINS dir step m0 m1 m2 enable");
  Serial.println("RUN FWD|REV rpm microsteps ramp_ms gear_ratio");
  Serial.println("MOVE FWD|REV degrees rpm microsteps ramp_ms gear_ratio");
  Serial.println("STOP | ESTOP | SPEED rpm | DIR FWD|REV | MICROSTEP n | GEAR ratio");
  Serial.println("Servo commands:");
  Serial.println("SERVOATTACH pin min_us max_us freq | SERVORANGE degrees");
  Serial.println("SERVOANGLE degrees move_ms | SERVOPULSE us | SERVOSWEEP min max period_ms");
  Serial.println("SERVOSTOP | SERVODETACH | SERVOSTATUS | STATUS");
  printStatus();
}

void loop() {
  readSerialCommands();
  const uint32_t nowUs = micros();
  maybeStep(nowUs);

  const uint32_t nowMs = millis();
  updateServoMove(nowMs);
  updateServoSweep(nowMs);
  if (nowMs - lastStatusMs >= kStatusIntervalMs) {
    lastStatusMs = nowMs;
    printStatus();
  }
}
