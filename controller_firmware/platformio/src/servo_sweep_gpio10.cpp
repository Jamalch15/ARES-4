#include <Arduino.h>

// Use the CH343 USB-UART bridge exposed as COMx on this board.
#define SERVO_TEST_SERIAL Serial0

// Minimal standalone servo sweep test for ESP32-S3.
// Wire servo signal to GPIO10, servo power to an external 5V/6V supply,
// and servo ground to ESP GND. Do not power a servo from ESP 3V3.

namespace {
constexpr int kServoPin = 10;
constexpr int kServoChannel = 0;
constexpr int kServoFrequencyHz = 50;
constexpr int kServoResolutionBits = 14;
constexpr uint32_t kServoMaxDuty = (1UL << kServoResolutionBits) - 1UL;

constexpr int kMinPulseUs = 700;
constexpr int kMaxPulseUs = 2300;
constexpr int kStepPulseUs = 8;
constexpr int kStepDelayMs = 20;

uint32_t dutyForPulseUs(int pulseUs) {
  const float periodUs = 1000000.0f / static_cast<float>(kServoFrequencyHz);
  const float duty = constrain(static_cast<float>(pulseUs) / periodUs, 0.0f, 1.0f);
  return static_cast<uint32_t>(roundf(duty * static_cast<float>(kServoMaxDuty)));
}

void writeServoPulse(int pulseUs) {
  ledcWrite(kServoChannel, dutyForPulseUs(pulseUs));
}
}  // namespace

void setup() {
  SERVO_TEST_SERIAL.begin(115200);
  delay(300);

  ledcSetup(kServoChannel, kServoFrequencyHz, kServoResolutionBits);
  ledcAttachPin(kServoPin, kServoChannel);
  writeServoPulse((kMinPulseUs + kMaxPulseUs) / 2);

  SERVO_TEST_SERIAL.println("SERVO_SWEEP_GPIO10 ready");
  SERVO_TEST_SERIAL.printf("pin=%d freq=%d min_us=%d max_us=%d\r\n", kServoPin, kServoFrequencyHz, kMinPulseUs, kMaxPulseUs);
}

void loop() {
  for (int pulse = kMinPulseUs; pulse <= kMaxPulseUs; pulse += kStepPulseUs) {
    writeServoPulse(pulse);
    delay(kStepDelayMs);
  }
  delay(250);
  for (int pulse = kMaxPulseUs; pulse >= kMinPulseUs; pulse -= kStepPulseUs) {
    writeServoPulse(pulse);
    delay(kStepDelayMs);
  }
  delay(250);
}
