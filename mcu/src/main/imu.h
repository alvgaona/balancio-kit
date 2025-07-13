#pragma once

/**
 * Inertial measurement unit (IMU) configuration and initialization.
 */
void imuSetup();

/**
 * Get acceleration values in Y and Z axis, and angular velocity in X and Z
 * axis.
 *
 * @param  {float*} ay : Pointer to store acceleration in Y axis.
 * @param  {float*} az : Pointer to store acceleration in Z axis.
 * @param  {float*} gx : Pointer to store angular velocity in X axis.
 * @param  {float*} gz : Pointer to store angular velocity in Z axis.
 */
void getAccelGyro(float *ay, float *az, float *gx, float *gz);

/**
 * Get robot's pitch based on accelerometer data.
 *
 */
float getAccelPitch();

/**
 * Estimate robot's pitch angle (in radians) based on accelerometer and
 * gyroscope data, using a complementary filter.
 *
 * @param  {float} currentAngle : Current pitch angle (in radians).
 * @return {float}              : New updated pitch angle (in radians).
 */
float updatePitch(float current_angle);

/**
 * Estimate robot's yaw (in radians), integrating the gyroscope data.
 *
 * @param  {float} currentYaw : Current yaw angle (in radians).
 * @return {float}            : New updated yaw angle (in radians).
 */
float updateYaw(float curent_yaw);
