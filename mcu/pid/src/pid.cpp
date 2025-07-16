#include "pid.h"
#include "config.h"
#include <algorithm>

PID::PID(float kp, float ki, float kd, float sumConstraint) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  sumConstraint_ = sumConstraint;
}

float PID::update(float currentState, float targetState) {
  error_ = currentState - targetState;
  errorSum_ += error_;
  errorSum_ = std::max(-sumConstraint_, std::min(sumConstraint_, errorSum_));
  out = kp_ * (error_) + ki_ * (errorSum_)*LOOP_PERIOD +
        kd_ * (currentState - prevState_) / LOOP_PERIOD;
  prevState_ = currentState;
  return out;
}

void PID::reset(float previousState) {
  errorSum_ = 0.0;
  prevState_ = previousState;
}
