#include "config.h"
#include "imu.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

void Imu::Imu() {}

void Imu::~Imu() {}

void Imu::setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu_.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // supply your own offsets here.
  mpu_.setXAccelOffset(X_ACCEL_OFFSET);
  mpu_.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu_.setZAccelOffset(Z_ACCEL_OFFSET);
  mpu_.setXGyroOffset(X_GYRO_OFFSET);
  mpu_.setYGyroOffset(Y_GYRO_OFFSET);
  mpu_.setZGyroOffset(Z_GYRO_OFFSET);
}

void Imu::getAccelGyro(float *ay, float *az, float *gx, float *gz) {
  // gyro (+/- 250 deg/s) accel (+/- 2g)
  #ifdef BALANCIO_BOARD
  // Rotate coordinates (swap y and z, invert y) to compensate for rotated MPU6050
  mpu.getMotion6(&ax_b, &az_b, &ay_b, &gx_b, &gz_b, &gy_b);
  ay_b *= -1.0;
  gy_b *= -1.0;
  #else
  mpu.getMotion6(&ax_b, &ay_b, &az_b, &gx_b, &gy_b, &gz_b);
  #endif

  ay_ = 4 * (ay_b / 65535.0);                             // g
  az_ = 4 * (az_b / 65535.0);                             // g
  gx_ = 2 * 250 * (gx_b / (65535.0));                     // deg/s
  gz_ = 2 * 250 * (gz_b / (65535.0));                     // deg/s
}

float Imu::getAccelPitch() {
  getAccelGyro(&ay_, &az_, &gx_, &gz_);
  return atan2(ay_, az_);
}

float Imu::updatePitch(float currentAngle) {
  float currentAngleDeg = -currentAngle * RAD_TO_DEG;

  getAccelGyro(&ay, &az, &gx, &gz);
  accelPitch_ = getAccelPitch() * RAD_TO_DEG;

  // Complementary filter between acceleration and gyroscopic pitch estimation.
  pitchDeg_ = (tau) * (currentAngleDeg + gx * LOOP_PERIOD) + (1 - tau_) * (accelPitch_);
  pitchRad_ = -pitchDeg_ * DEG_TO_RAD;

  return pitchRad;
}

float updateYaw(float currentYaw) {
  getAccelGyro(&ay, &az, &gx, &gz);

  // Integrate angular velocity to estimate the yaw angle
  return currentYaw + gz * DEG_TO_RAD;
}
