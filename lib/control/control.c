/*
System control module.
*/

#include <stdint.h>
#include <stdio.h>

#include "control.h"

// Gloabal system initialization.
struct System sys = {
    .controllers =  {
        {0.3f, 0.15f, 0.05f},
        {0.3f, 0.15f, 0.05f},
        {0.3f, 0.15f, 0.05f},
    },
    .setpoint = {0.0f, 0.0f, 2.0f, 0.0f},
    .ctrl_bound = {
        {0.0f, 2000.0f},
        {0.0f, 2000.0f},
        {0.0f, 2000.0f},
        {0.0f, 2000.0f},
    },
    .err_weights = {
        {1.0f, 0.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f},
    }
};

// Scalar product implementation.
static inline float scalarProduct(const float x[static STATE_DIM], const float y[static STATE_DIM])
{
    // return x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3];
    float res = 0;
    for (uint8_t i = 0; i < STATE_DIM; ++i)
        res += x[i] * y[i];
    return res;
}

// Set variable in specified lower and upper bound.
static inline float setBounds(const float x, const float bound[static 2])
{
    return x < bound[0] ? bound[0] : (x > bound[1] ? bound[1] : x);
}

// Compute error for a given state and setpoint.
static inline void computeError(struct System* sys, float error[static STATE_DIM])
{
    static float temp_err[STATE_DIM];
    // Compute state errors as the weighted sum
    // of setpoint and state components difference.
    for (uint8_t i = 0; i < STATE_DIM; ++i)
        temp_err[i] = sys->setpoint[i] - sys->state[i];

    for (uint8_t i = 0; i < STATE_DIM; ++i)
        error[i] = scalarProduct(err_weights[i], temp_err);
}

void updateControl(struct System* sys)
{
    static float error[STATE_DIM]; // weighted error vector
    static float err_integral[STATE_DIM]; // integral error terms
    static float prev_err[STATE_DIM]; // previous state error to compute derivative term

    computeError(sys, error);

    for (uint8_t i = 0; i < STATE_DIM; ++i) // update integral terms
        err_integral[i] = setBounds(err_integral[i] + error[i], sys->ctrl_bound[i]);

    for (uint8_t i = 0; i < STATE_DIM; ++i) // update control vector
        sys->control[i] = setBounds(sys->controllers[i].k_p * error[i]
                + sys->controllers[i].k_i * err_integral[i]
                + sys->controllers[i].k_d * (error[i] - prev_err[i]),
            sys->ctrl_bound[i]);

    if (!(sys->time_cnt++ % DER_FILTER_ETA)) // update time counter
        for (uint8_t i = 0; i < STATE_DIM; ++i) // update previous state error
            prev_err[i] = error[i];
};

void test()
{
    printf("%f\t%f\t%f\n", sys.state[2], sys.controllers[1].k_i, sys.ctrl_bound[3][1]);

    /* // Main loop will look like this:
    while (1) {
        if (!checkNewData())
            continue;
        getSensorData();
        updateState();
        updateControl();
        sendControl();
        writeDataBuffer();
        if (isFull(data_buffer))
            sendDataBuffer();
    }
    */
}