/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/
#ifndef CONFIG_H
#define CONFIG_H

#define NEW_BOARD

// PID Constants
#ifdef NEW_BOARD
#define KP_PITCH 1500
#define KI_PITCH 5000
#define KD_PITCH 20.0
#define KP_YAW 5.0
#define KI_YAW 0.0
#define KD_YAW 0.05
#else
#define KP_PITCH 2000
#define KI_PITCH 22000
#define KD_PITCH 20.0
#define KP_YAW 2.0
#define KI_YAW 0.0
#define KD_YAW 0.05
#endif

// Loop period in seconds (100 Hz)
#define LOOP_PERIOD 0.01 

// Calibrated point
#define STATIC_ANGLE -0.04

// Accelerometer offsets
#define X_ACCEL_OFFSET -291
#define Y_ACCEL_OFFSET  -3307
#define Z_ACCEL_OFFSET  961
#define X_GYRO_OFFSET   -49
#define Y_GYRO_OFFSET   1
#define Z_GYRO_OFFSET   27

#define PIN_ENA 23
#define PIN_IN1 22
#define PIN_IN2 21
#define PIN_IN3 19
#define PIN_IN4 18
#define PIN_ENB 5

#define PIN_SDA 26
#define PIN_SCL 27

#define PIN_ENABLE_BUTTON 0

// Only for new board
#define PIN_LEDR 13
#define PIN_LEDG 14
#define PIN_LEDB 12

#endif
