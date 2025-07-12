/* 
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
====================================================================== 
*/

#include "dc_motor.h"
#include "config.h"

#define L_CHANNEL 0
#define R_CHANNEL 1

const int PWM_freq = 200;

void motor_init(void)
{
  // Pins configuration
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_IN3, OUTPUT);
  pinMode(PIN_IN4, OUTPUT);

  // Start with stopped motor
  stop_motor();

  // PWM
  ledcAttachPin(PIN_ENA, L_CHANNEL);     // (Pin, Channel)
  ledcAttachPin(PIN_ENB, R_CHANNEL);     // (Pin, Channel)
  ledcSetup(L_CHANNEL, PWM_freq, 8); // (Channel, PWM frequency, bits resolution)
  ledcSetup(R_CHANNEL, PWM_freq, 8); // (Channel, PWM frequency, bits resolution)
  ledcWrite(L_CHANNEL, 0);           // (channel, bits)
  ledcWrite(R_CHANNEL, 0);           // (channel, bits)
}

void stop_motor(void)
{
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, LOW);
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, LOW);
}

void bwd_L_motor(int pwm)
{
  // PWM 8 bits int --> [0; 255]
  digitalWrite(PIN_IN1, HIGH);
  digitalWrite(PIN_IN2, LOW);
  ledcWrite(L_CHANNEL, pwm);
}

void bwd_R_motor(int pwm)
{
  // PWM 8 bits int --> [0; 255]
  digitalWrite(PIN_IN3, LOW);
  digitalWrite(PIN_IN4, HIGH);
  ledcWrite(R_CHANNEL, pwm);
}

void fwd_L_motor(int pwm)
{
  // PWM 8 bits int --> [0; 255].
  digitalWrite(PIN_IN1, LOW);
  digitalWrite(PIN_IN2, HIGH);
  ledcWrite(L_CHANNEL, pwm);
}

void fwd_R_motor(int pwm)
{
  // PWM 8 bits int --> [0; 255].
  digitalWrite(PIN_IN3, HIGH);
  digitalWrite(PIN_IN4, LOW);
  ledcWrite(R_CHANNEL, pwm);
}

void R_motor(int pwm)
{
  pwm = constrain(pwm, -255.0, 255.0);
  if (pwm >= 0)
  {
    fwd_R_motor(pwm);
  }
  else
  {
    bwd_R_motor(-pwm);
  }
}

void L_motor(int pwm)
{
  pwm = constrain(pwm, -255.0, 255.0);
  if (pwm >= 0)
  {
    fwd_L_motor(pwm);
  }
  else
  {
    bwd_L_motor(-pwm);
  }
}
