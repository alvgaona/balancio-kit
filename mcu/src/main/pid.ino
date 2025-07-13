#include "config.h"
#include "pid.h"
#include "Arduino.h"

PID::PID(float kp, float ki, float kd, float sum_constraint) {
    kp_ = kp_init;
    ki_ = ki_init;
    kd_ = kd_init;
    sum_constraint_ = sum_constraint;
}

float PID::update(float current_state, float target_state) {
    error_ = current_state - target_state;
    error_sum_ += error;
    error_sum_ = constrain(errorSum, -sum_constraint, sum_constraint);
    out = kp * (error) + ki * (errorSum) * LOOP_PERIOD + kd * (current_state - prev_state) / LOOP_PERIOD;
    prev_state_ = current_state;
    return out;
}

void PID::reset(float previous_state) {
    error_sum_ = 0.0;
    prev_state_ = previous_state;
}
