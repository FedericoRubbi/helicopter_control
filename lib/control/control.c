/*
System control module.
*/

#include <stdint.h>
#include <stdio.h>

#include "control.h"
#include "ins/ins.h"

// Global system initialization.
struct System sys = {
    .controllers =  {
        {0.3f, 0.15f, 0.05f},
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
    .ctrl_dependence = { // it will be something similar to this matrix
        {1.0f, 1.0f, 0.0f, 0.0f},
        {1.0f, -1.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 1.0f, 1.0f},
        {0.0f, 0.0f, 1.0f, -1.0f},
    }
};

// Fixed-size fast scalar product implementation.
static inline float scalarProduct(const float x[static STATE_DIM], const float y[static STATE_DIM])
{
    return x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3];
}

// Set variable in specified lower and upper bound.
static inline float setBounds(const float x, const float bound[static 2])
{
    return x < bound[0] ? bound[0] : (x > bound[1] ? bound[1] : x);
}

// TODO: consider implementing derivative average.
void updateControl(struct System* sys)
{
    static float error[STATE_DIM]; // weighted error vector
    static float err_integral[STATE_DIM]; // integral error terms
    static float prev_err[STATE_DIM]; // previous state error to compute derivative term
    static float state_control[STATE_DIM]; // PID state control

    // Iterate PID computations
    for (uint8_t i = 0; i < STATE_DIM; ++i) {
        error[i] = sys->setpoint[i] - sys->state[i]; // compute error

        // Update integral term.
        err_integral[i] = setBounds(err_integral[i] + error[i], sys->ctrl_bound[i]);

        state_control[i] = setBounds(sys->controllers[i].k_p * error[i] // update control
                + sys->controllers[i].k_i * err_integral[i] * INS_SAMPLE_TIME
                + sys->controllers[i].k_d * (error[i] - prev_err[i]),
            sys->ctrl_bound[i]);
    }

    for (uint8_t i = 0; i < STATE_DIM; ++i)
        sys->control[i] = scalarProduct(sys->ctrl_dependence[i], state_control);


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