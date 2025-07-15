#pragma once

#include "MPU6050_6Axis_MotionApps_V6_12.h"

class Imu {
public:
  Imu();
  ~Imu();

  void setup();
  void getAccelGyro(float *ay, float *az, float *gx, float *gz);
  float getAccelPitch();
  float updatePitch(float current_angle);
  float updateYaw(float current_yaw);

private:
  MPU6050 mpu_;
  float az_;
  float ay_;
  float gx_;
  float gy_;
  float gz_;
  float tau_ = 0.98; // Complementary filter coefficient

  float accelPitch_;
  float pitchDeg_;
  float pitchRad_;

  int16_t ax_b;
  int16_t ay_b;
  int16_t az_b;
  int16_t gx_b;
  int16_t gy_b;
  int16_t gz_b;
};
