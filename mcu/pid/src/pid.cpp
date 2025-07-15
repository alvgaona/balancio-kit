#include "config.h"
#include "pid.h"
#include "Arduino.h"

PID::PID(float kp, float ki, float kd, float sumConstraint) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    sumConstraint_ = sumConstraint;
}

float PID::update(float currentState, float targetState) {
    error_ = currentState - targetState;
    errorSum_ += error_;
    errorSum_ = constrain(errorSum_, -sumConstraint_, sumConstraint_);
    out = kp_ * (error_) + ki_ * (errorSum_) * LOOP_PERIOD + kd_ * (currentState - prevState_) / LOOP_PERIOD;
    prevState_ = currentState;
    return out;
}

void PID::reset(float previousState) {
    errorSum_ = 0.0;
    prevState_ = previousState;
}
