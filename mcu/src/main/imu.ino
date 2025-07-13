#include "config.h"
#include "imu.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

// Gyro, accel and euler vars
int16_t ax_b, ay_b, az_b, gx_b, gy_b, gz_b;

// Pitch calculator vars
float ay, az, gx, gz;
float accelPitch, pitchDeg, pitchRad;
float tau = 0.98; // Complementary filter parameter.
float newYaw;

// Functions
void imuSetup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin(PIN_SDA, PIN_SCL, 400000);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // supply your own offsets here.
  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET);
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);
}

void getAccelGyro(float *ay, float *az, float *gx, float *gz)
{
  // gyro (+/- 250 deg/s) accel (+/- 2g)
  #ifdef NEW_BOARD
  // Rotate coordinates (swap y and z, invert y) to compensate for rotated MPU6050
  mpu.getMotion6(&ax_b, &az_b, &ay_b, &gx_b, &gz_b, &gy_b);
  ay_b *= -1.0;
  gy_b *= -1.0;
  #else
  mpu.getMotion6(&ax_b, &ay_b, &az_b, &gx_b, &gy_b, &gz_b);
  #endif

  ay[0] = 4 * (ay_b / 65535.0);                             // g
  az[0] = 4 * (az_b / 65535.0);                             // g
  gx[0] = 2 * 250 * (gx_b / (65535.0));                     // deg/s
  gz[0] = 2 * 250 * (gz_b / (65535.0));                     // deg/s
}

float getAccelPitch() {
  getAccelGyro(&ay, &az, &gx, &gz);
  float accelPitch = atan2(ay, az);
  return accelPitch;
}

float updatePitch(float currentAngle)
{
  float currentAngleDeg = -currentAngle * RAD_TO_DEG;

  getAccelGyro(&ay, &az, &gx, &gz);
  accelPitch = getAccelPitch() * RAD_TO_DEG;

  // Complementary filter between acceleration and gyroscopic pitch estimation.
  pitchDeg = (tau) * (currentAngleDeg + gx * LOOP_PERIOD) + (1 - tau) * (accelPitch);
  pitchRad = -pitchDeg * DEG_TO_RAD;

  return pitchRad;
}

float updateYaw(float currentYaw)
{
  // Get imu data.
  getAccelGyro(&ay, &az, &gx, &gz);
  // Yaw estimation integrating angular velocity.
  newYaw = currentYaw + gz * DEG_TO_RAD;

  return newYaw;
}
