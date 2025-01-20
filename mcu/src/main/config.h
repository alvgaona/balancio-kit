/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/
#pragma once

// PID Constants for pitch control
#define KP 2000  // 2000     //1200    //1750
#define KI 22000 // 22000     //22000   //11000
#define KD 20.0  // 20.0     //17.0    //25.0

// PID Constants for yaw control
#define KP_YAW 5.0
#define KI_YAW 0.0
#define KD_YAW 0.05

// Loop period in seconds
#define LOOP_PERIOD 0.01 // 100 Hz

#define STATIC_ANGLE -0.02 // Calibrated point

// IMU calibration parameters
#define X_ACCEL_OFFSET -3026
#define Y_ACCEL_OFFSET  547
#define Z_ACCEL_OFFSET  1635
#define X_GYRO_OFFSET   80
#define Y_GYRO_OFFSET   34
#define Z_GYRO_OFFSET   -69

