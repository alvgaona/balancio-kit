#pragma once

/**
 * @brief PID (Proportional-Integral-Derivative) Controller implementation.
 *
 * This class implements a discrete PID controller that can be used for feedback
 * control systems. The controller calculates an output based on the error
 * between a target state and current state using proportional, integral, and
 * derivative terms.
 *
 * The controller equation is: output = Kp*error + Ki*∑error + Kd*(error -
 * prev_error)
 */
class PID {
private:
  float error_;           ///< Current error value (target - current)
  float errorSum_ = 0.0;  ///< Accumulated error sum for integral term
  float prevState_ = 0.0; ///< Previous state value for derivative calculation
  float kp_, ki_, kd_;    ///< PID gains (proportional, integral, derivative)
  float
      sumConstraint_; ///< Maximum absolute value for integral sum (anti-windup)
  int out;            ///< Controller output value

public:
  /**
   * @brief Constructs a PID controller with specified gains and constraints.
   *
   * Initializes the PID controller with the given parameters. The
   * sum_constraint parameter helps prevent integral windup by limiting the
   * accumulated error sum.
   *
   * @param kp Proportional gain - determines response to current error
   * @param ki Integral gain - determines response to accumulated error
   * @param kd Derivative gain - determines response to rate of error
   * change
   * @param sumConstraint Maximum absolute value for integral sum to
   * prevent windup
   */
  PID(float kp, float ki, float kd, float sumConstraint);

  /**
   * @brief Updates the PID controller and calculates the control output.
   *
   * Computes the PID control output based on the current error between target
   * and current states. Updates internal state variables for next iteration.
   *
   * @param current_state The current measured value of the system
   * @param target_state The desired setpoint value
   * @return The calculated control output value
   */
  float update(float currentState, float targetState);

  /**
   * @brief Resets the PID controller's internal state.
   *
   * Clears the integral accumulator and sets the previous state for derivative
   * calculation. This should be called when starting control or after a
   * significant disturbance to prevent erratic behavior.
   *
   * @param previousState The state value to use as the previous state for
   *                      derivative calculation on the next update
   */
  void reset(float previousState);
};
