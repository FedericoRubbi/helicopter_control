#ifndef CTRL_H
#define CTRL_H

#include <stdint.h>

#define STATE_DIM 4

// TODO: consider implementing derivative average.
// Derivative interval time units for noise reduction.
// Set to a power of 2 smaller than 256 for efficiency.
#define DER_FILTER_ETA 4

struct PidController {
    const float k_p; // proportional coefficient
    const float k_i; // integral coefficient
    const float k_d; // derivative coefficient
} PidController;

struct System {
    // State vector containing in order x, y, z coordinates and yaw.
    float state[STATE_DIM];
    // State setpoint vector.
    float setpoint[STATE_DIM];
    // Physical control vector containing in order lower and upper motor, left and right servo.
    float control[STATE_DIM];
    // Control lower and upper boundaries.
    const float ctrl_bound[STATE_DIM][2];
    // Pid controllers associated to control vector.
    const struct PidController controllers[STATE_DIM];
    // Matrix for the change of basis from PID state controls to physical controls.
    float ctrl_dependence[STATE_DIM][STATE_DIM];
    // Time counter for derivative filter.
    uint8_t time_cnt;
};

// Fast scalar product for fixed-size arrays.
static inline float scalarProduct(const float x[static STATE_DIM], const float y[static STATE_DIM]);
static inline float setBounds(float x, const float bound[static 2]);
void updateControl();

#endif // CTRL_H