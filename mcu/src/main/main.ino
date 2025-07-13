/*
  ============================================
  Balancio-Kit is placed under the MIT License
  Copyright (c) 2021 by Linar (UdeSA)

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  ===============================================
*/

#include "config.h"
#include "dc_motor.h"
#include "controllers.h"
#include "imu.h"
#include "timer.h"
#include "Wire.h"
#include "App.h"
volatile bool controlFlag = false;

float currentPitch = 0.0, targetPitch = 0.0;
const float pitchLimit = 0.5;

float currentYaw = 0.0, targetYaw = 0.0;

int pwmL = 0, pwmR = 0;
int timeControl;
bool prevState = false;
bool currState = false;

PID *yawControl;
PID *pitchControl;
float pwm;
float yawOutput;
App* inputDevice;
float inputPoint[2];

bool enabled = false;

void setup() {
  // UART PC connection
  Serial.begin(115200);
  // Motor initialization
  motorInit();
  // IMU init
  imuSetup();
  inputDevice = new App();
  // Bluetooth init
  inputDevice->setup();

  // Timer init. (ISR at 1/LOOP_PERIOD Hz)
  timerInit();

  // Controllers init
  pitchControl = new PID(KP_PITCH, KI_PITCH, KD_PITCH, 5.0);
  yawControl = new PID(KP_YAW, KI_YAW, KD_YAW, 5.0);

  // Angle initialization
  currentPitch = -getAccelPitch();
  timeControl = micros();

  pinMode(PIN_LEDR, OUTPUT);
  pinMode(PIN_LEDG, OUTPUT);
  pinMode(PIN_LEDB, OUTPUT);
  pinMode(PIN_ENABLE_BUTTON, INPUT_PULLUP);
}

// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime() {
  controlFlag = true;
}

void loop() {
  if (!controlFlag){
    return;
  }
  controlFlag = false;

  if (digitalRead(PIN_ENABLE_BUTTON) == 0) {
    enabled = !enabled;
    delay(200);
  }
  if (!enabled) {
    stopMotor();
    digitalWrite(PIN_LEDR, LOW);
    return;
  }

  // Get pitch and yaw angles.
  currentPitch = updatePitch(currentPitch);
  currentYaw = updateYaw(currentYaw);

  if (currentPitch > pitchLimit || currentPitch < -pitchLimit) {
    currState = false;
    stopMotor();
    targetYaw = currentYaw;

    // Reset PID
    pitchControl->reset(0.0);
    yawControl->reset(targetYaw);
    digitalWrite(PIN_LEDR, HIGH);

  } else {
    currState = true;

    // Pitch control
    pwm = pitchControl->update(currentPitch, targetPitch);
    pwmL = pwm;
    pwmR = pwm;

    // Yaw control
    yawOutput = yawControl->update(currentYaw, targetYaw);

    // Pass PWM commands to motors.
    leftMotor(pwmL + int(yawOutput)); // -255 to 255
    rightMotor(pwmR - int(yawOutput)); // -255 to 255

    digitalWrite(PIN_LEDR, LOW);
  }
  if (true) {
    // Print relevant data.
    Serial.print("  targetPitch: ");
    Serial.print(targetPitch);
    Serial.print("  targetYaw: ");
    Serial.print(targetYaw);
    Serial.print(" pitch: ");
    Serial.print(currentPitch);
    Serial.print(" yaw: ");
    Serial.print(currentYaw);
    Serial.print(" Loop time: ");
    Serial.println(micros() - timeControl);

    timeControl = micros();
  }
  inputDevice->parseInput(inputPoint);

  // Get joystick commands.
  targetPitch = inputDevice->getPitchCommand(inputPoint[0]);
  targetYaw = inputDevice->getYawCommand(targetYaw,inputPoint[1]);

  if(prevState != currState){
    stoppedCommand(currState);
  }
  prevState = currState;
}
