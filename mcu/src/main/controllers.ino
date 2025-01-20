/* 
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
====================================================================== 
*/

#include "config.h"
#include "controllers.h"
#include "Arduino.h"

/*PID Class*/

PID::PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init)
{
    kp = kp_init;
    ki = ki_init;
    kd = kd_init;
    sum_constraint = sum_constraint_init; 
}

float PID::update(float current_state, float target_state)
{
    error = current_state - target_state;
    errorSum += error;
    errorSum = constrain(errorSum, -sum_constraint, sum_constraint);
    out = kp * (error) + ki * (errorSum)*LOOP_PERIOD + kd * (current_state - prev_state) / LOOP_PERIOD;
    prev_state = current_state;
    return out;
}

void PID::reset(float previous_state)
{
    errorSum = 0.0;
    prev_state = previous_state;
}
