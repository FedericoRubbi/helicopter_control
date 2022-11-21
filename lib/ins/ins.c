#include "hardware/i2c.h"

#include "ins.h"
#include "imu.h"

// TODO: implement acceleration software calibration.
// TODO: consider checking and correcting unit quaternion norm.
// Fast formula to subtract gravity.
// Gravity is assumed to be *only* on vertical z-axis, hence good angle calibration is required.
static inline void filter_gravity(struct INS_s* ins)
{
    double* q = ins->imu.quaternion;
    double* a = ins->imu.acceleration;
    // double norm_sq = quat_norm(q); // compute square of the norm
    // norm_sq *= norm_sq;
    a[0] += 2.0 * (q[1] * q[3] - q[0] * q[2]) /* / norm_sq*/;
    a[1] += 2.0 * (q[0] * q[1] + q[2] * q[3]) /* / norm_sq*/;
    a[2] += q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3] /* / norm_sq*/;
}

// Compute velocity and position with trapezoidal rule integration.
void update_state(struct INS_s* ins)
{
    filter_gravity(ins);
    // Integrate acceleration to compute position.
    for (uint8_t i = 0; i < 3; ++i) {
        ins->velocity[i] += ins->imu.acceleration[i] * IMU_SAMPLE_TIME * INS_SCALE_VELOCITY;
        ins->position[i] += ins->velocity[i] * IMU_SAMPLE_TIME;
    }
}
