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
float currentAngle, targetAngle = 0.0, angle_limit = 0.5;
float yaw = 0.0, targetYaw = 0.0, yawCommand = 0.0;
int pwmL = 0, pwmR = 0;
volatile bool controlFlag = false;
float fwd = 0;
float rot = 0;
int time_ctr1, time_ctr2;
int delay_startup_time = 2000;
bool x_down;

controller_data_t pitch_controller_data = {KP, KI, KD};
controller_data_t yaw_controller_data = {KP_YAW, KI_YAW, KD_YAW};

PID *yaw_control;
PID *pitch_control;
float pwm;
float rot_v;
App* inp_device; 
float p_y[2];

// ISR at 1/LOOP_PERIOD H
void setup()
{
  // UART PC connection
  Serial.begin(115200);
  // Motor initialization
  motor_init();
  // IMU init
  imu_setup();
  inp_device = new App();
  // Bluetooth init
  inp_device->setup();

  // Timer init. (ISR at 1/LOOP_PERIOD Hz)
  timer_init();
  time_ctr1 = micros();
  time_ctr2 = millis();

  // Controllers init
  pitch_control = new PID(pitch_controller_data.kp, pitch_controller_data.ki, pitch_controller_data.kd, 5.0);
  yaw_control = new PID(yaw_controller_data.kp, yaw_controller_data.ki, yaw_controller_data.kd, 5.0);

  // Angle initialization
  currentAngle = -getAccelPitch();
}


// ISR at 1/LOOP_PERIOD Hz
void IRAM_ATTR onTime()
{
  controlFlag = true;
}

void loop()
{

  if (controlFlag)
  {
    // Get pitch and yaw angles.
    currentAngle = updatePitch(currentAngle);
    yaw = updateYaw(yaw);

    if ((currentAngle > angle_limit) || (currentAngle < -angle_limit))
    {
      // Stop motors
      stop_motor();

      // Reset yaw command
      targetYaw = yaw;

      // Reset PID
      pitch_control->reset(0.0);
      yaw_control->reset(targetYaw);
    }
    else
    {
      // Pitch control

      // PID control
      pwm = pitch_control->update(currentAngle, targetAngle);
      pwmL = pwm;
      pwmR = pwm;

      // Yaw control
      rot_v = yaw_control->update(yaw, targetYaw);
      rot = rot_v;

      // Pass PWM commands to motors.
      L_motor(pwmL + int(rot)); // -255 to 255
      R_motor(pwmR - int(rot)); // -255 to 255
    }
    // Print relevant data.
    if (true)
    {
        Serial.print("  targetAngle: ");
        Serial.print(targetAngle);
        Serial.print("  targetYaw: ");
        Serial.print(targetYaw);
        Serial.print(" pitch: ");
        Serial.print(currentAngle);
        Serial.print("  PWM: ");
        Serial.print(pwmL);
        Serial.print(" Loop time: ");
        Serial.println(micros() - time_ctr1);
      
        time_ctr1 = micros();
    }
    controlFlag = false;
  }
  else
  {
    return;
  }
  
  inp_device->parse_input(p_y);

  // Get joystick commands.
  targetAngle = inp_device->get_pitch_command(p_y[0]);
  targetYaw = inp_device->get_yaw_command(targetYaw,p_y[1]);
}
int main() {
  setup();

  while (true) {
    loop();
  }

  return 0;
}
