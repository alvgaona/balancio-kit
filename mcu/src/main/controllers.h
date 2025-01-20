/*
======================================================================
 Balancio-Kit (c) 2021 Linar (UdeSA)
 This code is licensed under MIT license (see LICENSE.txt for details)
======================================================================
*/

#ifndef controllers_h
#define controllers_h
#include "config.h"

/**
 * Struct datatype to store PID related parameters.
 */
struct controller_data_t
{
    float kp;
    float ki;
    float kd;
};

class PID
{
private:
    float error;
    float errorSum = 0.0;
    float prev_state = 0.0;
    float kp, ki, kd;
    float sum_constraint;
    int out;

public:
    /**
     * Class constructor.
     *
     * Controller specific parameters are loaded.
     *
     * @param  {float} kp_init             : Proportional parameter.
     * @param  {float} ki_init             : Integral parameter.
     * @param  {float} kd_init             : Derivative parameter.
     * @param  {float} sum_constraint_init : Integral constraint, to avoid overflow.
     */
    PID(float kp_init, float ki_init, float kd_init, float sum_constraint_init);

    /**
     * PID controller update and output.
     *
     * @param  {float} current_state : Current state.
     * @param  {float} target_state  : Target state.
     * @return {float}  : Controller output.
     */
    float update(float current_state, float target_state);

    /**
     * Reset PID controller computation.
     * It restarts the integral and derivative components.
     *
     * @param  {float} previous_state : Current state
     */
    void reset(float previous_state);
};

#endif
