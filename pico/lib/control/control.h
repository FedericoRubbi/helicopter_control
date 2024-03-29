#ifndef CTRL_H
#define CTRL_H

#include <stdint.h>
#include "actuator.h"
#include "ins/ins.h"

#define CTRL_STATE_DIM 4 // state and control vectors dimension
#define CTRL_PID_FREQ ACT_S_FREQ

// TODO: consider implementing derivative average.
// Derivative interval time units for noise reduction.
// Set to a power of 2 smaller than 256 for efficiency.
#define CTRL_DER_FILTER_ETA 2

struct PidController_s
{
    float k_p; // proportional coefficient
    float k_i; // integral coefficient
    float k_d; // derivative coefficient
} __attribute__((packed));

struct System_s
{
    // State vector containing acceleration on x, y axes, z coordinate (height) and yaw.
    float state[CTRL_STATE_DIM];
    // State setpoint vector.
    float setpoint[CTRL_STATE_DIM];

    // Physical control vector containing servos and motor drivers duty cycles in ms.
    float control[CTRL_STATE_DIM];
    // Control lower and upper boundaries.
    const float ctrl_bound[CTRL_STATE_DIM][2];
    // Physical control actuators: in order right and left servo, lower and upper motor.
    struct Servo_s servo[2];
    struct Motor_s motor[2];

    // Pid controllers associated to control vector.
    struct PidController_s controllers[CTRL_STATE_DIM];
    // Matrix for the change of bases from state-related PID controls to physical controls.
    float ctrl_dependence[CTRL_STATE_DIM][CTRL_STATE_DIM];
    // Time counter for derivative filter.
    uint8_t time_cnt;
};

void setup_control(struct System_s *sys);
void stop(struct System_s *sys);
void takeoff(struct System_s *sys);
void compute_control(struct System_s *sys);
void update_control(struct System_s *sys, struct INS_s *ins);

#endif // CTRL_H