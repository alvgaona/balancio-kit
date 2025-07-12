/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

#ifndef IMU_H
#define IMU_H

/**
 * Inertial measurement unit (IMU) configuration and initialization.
 */
void imu_setup(void);

/**
 * Get acceleration values in Y and Z axis, and angular velocity in X and Z axis.
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
float getAccelPitch(void);

/**
 * Estimate robot's pitch angle (in radians) based on accelerometer and
 * gyroscope data, using a complementary filter.
 *
 * @param  {float} currentAngle : Current pitch angle (in radians).
 * @return {float}              : New updated pitch angle (in radians).
 */
float updatePitch(float currentAngle);

/**
 * Estimate robot's yaw (in radians), integrating the gyroscope data.
 *
 * @param  {float} currentYaw : Current yaw angle (in radians).
 * @return {float}            : New updated yaw angle (in radians).
 */
float updateYaw(float currentYaw);

#endif
