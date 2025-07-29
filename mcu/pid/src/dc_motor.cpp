#include "dc_motor.h"
#include "config.h"
#include <algorithm>
#include <esp32-hal-gpio.h>

#define L_CHANNEL 0
#define R_CHANNEL 1

Motors::Motors() {}

Motors::~Motors() { stop(); }

void Motors::init() const {
  // Pins configuration
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Start with stopped motor
  stop();

  // PWM
  ledcAttachPin(PIN_ENA, L_CHANNEL); // (Pin, Channel)
  ledcAttachPin(PIN_ENB, R_CHANNEL); // (Pin, Channel)
  ledcSetup(L_CHANNEL, pwmFrequency,
            8); // (Channel, PWM frequency, bits resolution)
  ledcSetup(R_CHANNEL, pwmFrequency,
            8);            // (Channel, PWM frequency, bits resolution)
  ledcWrite(L_CHANNEL, 0); // (channel, bits)
  ledcWrite(R_CHANNEL, 0); // (channel, bits)
}

void Motors::stop() {
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

void Motors::bwdLeftMotor(int pwm) {
  // PWM 8 bits int --> [0; 255]
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(L_CHANNEL, pwm);
}

void Motors::bwdRightMotor(int pwm) {
  // PWM 8 bits int --> [0; 255]
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  ledcWrite(R_CHANNEL, pwm);
}

void Motors::fwdLeftMotor(int pwm) {
  // PWM 8 bits int --> [0; 255].
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  ledcWrite(L_CHANNEL, pwm);
}

void Motors::fwdRightMotor(int pwm) {
  // PWM 8 bits int --> [0; 255].
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  ledcWrite(R_CHANNEL, pwm);
}

void Motors::rightMotor(int pwm) {
  // Limit pwm between -255 and 255 using min/max
  pwm = std::max(-255, std::min(pwm, 255));
  if (pwm >= 0) {
    fwdRightMotor(pwm);
  } else {
    bwdRightMotor(-pwm);
  }
}

void Motors::leftMotor(int pwm) {
  // Limit pwm between -255 and 255 using min/max
  pwm = std::max(-255, std::min(pwm, 255));
  if (pwm >= 0) {
    fwdLeftMotor(pwm);
  } else {
    bwdLeftMotor(-pwm);
  }
}
