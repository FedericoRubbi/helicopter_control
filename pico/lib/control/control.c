#include <stdint.h>
#include <stdio.h>

#include "control.h"
#include "actuator.h"
#include "ins/ins.h"

static inline float scalar_product(const float x[CTRL_STATE_DIM], const float y[CTRL_STATE_DIM]);
static inline float set_bounds(float x, const float bound[2]);
static void get_state(struct System_s *sys, struct INS_s *ins);

const float pid_integral_bounds[2] = {ACT_M_MAX, ACT_M_MAX};

// Fixed-size fast scalar product implementation.
static inline float scalar_product(const float x[static CTRL_STATE_DIM], const float y[static CTRL_STATE_DIM])
{
    return x[0] * y[0] + x[1] * y[1] + x[2] * y[2] + x[3] * y[3];
}

// Set variable in specified lower and upper bound.
static inline float set_bounds(const float x, const float bound[static 2])
{
    return x < bound[0] ? bound[0] : (x > bound[1] ? bound[1] : x);
}

void setup_control(struct System_s *sys)
{
    setup_servos(&sys->servo[0], &sys->servo[1]); // setup right and left servos
    setup_motors(&sys->motor[0], &sys->motor[1]); // setup lower and upper motors
}

// Stop procedure.
void stop(struct System_s *sys)
{
    sys->motor[0].duty = sys->motor[1].duty = ACT_M_STOP;
    update_motors(&sys->motor[0], &sys->motor[1]); // update lower and upper motors
}

// Power motors at minimum power to take off.
void takeoff(struct System_s *sys)
{
    sys->motor[0].duty = ACT_M_TAKEOFF;
    sys->motor[1].duty = ACT_M_TAKEOFF;
    update_motors(&sys->motor[0], &sys->motor[1]); // update lower and upper motors
}

// Get state from ins.
static void get_state(struct System_s *sys, struct INS_s *ins)
{
    sys->state[0] = (float)ins->imu.acceleration[0]; // acceleration on x axis
    sys->state[1] = (float)ins->imu.acceleration[1]; // acceleration on y axis
    sys->state[2] = (float)ins->position[2];         // height
    sys->state[3] = ins->imu.angle[2];               // yaw angle
}

// PID algorithm implementation.
void compute_control(struct System_s *sys)
{
    static float error[CTRL_STATE_DIM];         // weighted error vector
    static float err_integral[CTRL_STATE_DIM];  // integral error terms
    static float prev_err[CTRL_STATE_DIM];      // previous state error to compute derivative term
    static float state_control[CTRL_STATE_DIM]; // PID state control

    // Iterate PID computations
    for (uint8_t i = 0; i < CTRL_STATE_DIM; ++i)
    {
        error[i] = sys->setpoint[i] - sys->state[i]; // compute error

        // Update integral term.
        err_integral[i] = set_bounds(err_integral[i] + error[i], pid_integral_bounds);

        state_control[i] = sys->controllers[i].k_p * error[i] // update control
                           + sys->controllers[i].k_i * err_integral[i] / CTRL_PID_FREQ +
                           sys->controllers[i].k_d * (error[i] - prev_err[i]) * CTRL_PID_FREQ;
    }

    for (uint8_t i = 0; i < CTRL_STATE_DIM; ++i)
        sys->control[i] = set_bounds(scalar_product(sys->ctrl_dependence[i], state_control), sys->ctrl_bound[i]);

    if (!(sys->time_cnt++ % CTRL_DER_FILTER_ETA))    // update time counter
        for (uint8_t i = 0; i < CTRL_STATE_DIM; ++i) // update previous state error
            prev_err[i] = error[i];
};

// PID control procedure.
void update_control(struct System_s *sys, struct INS_s *ins)
{
    get_state(sys, ins);  // copy state vector
    compute_control(sys); // compute physical control vector
    // Copy physical control to actuator structs.
    sys->servo[0].duty_period = sys->control[0];
    sys->servo[1].duty_period = sys->control[1];
    sys->motor[0].duty = sys->control[2];
    sys->motor[1].duty = sys->control[3];

    update_servos(&sys->servo[0], &sys->servo[1]); // update right and left servos
    update_motors(&sys->motor[0], &sys->motor[1]); // update lower and upper motors
}
