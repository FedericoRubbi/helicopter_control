
#include <pico/stdlib.h>
#include <hardware/i2c.h>

#include "ins.h"
#include "imu.h"
#include "us_sensor.h"

static inline float abs(float x) { return x >= 0 ? x : -x; }

// Compute time interval in seconds.
static inline float delta(uint16_t ms0, uint16_t ms1) { return 0.001 * (float)(ms1 - ms0); }

// Setup i2c communication on specified pins.
void setup_ins(struct INS_s *ins)
{
    i2c_init(i2c1, INS_DEFAULT_BAUDRATE);
    gpio_set_function(INS_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(INS_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(INS_SDA_PIN);
    gpio_pull_up(INS_SCL_PIN);
}

// TODO: implement acceleration software calibration.
// TODO: consider checking and correcting unit quaternion norm.
// Fast formula to subtract gravity.
// Gravity is assumed to be *only* on vertical z-axis, hence good angle calibration is required.
static inline void filter_gravity(struct INS_s *ins)
{
    double *q = ins->imu.quaternion;
    double *a = ins->imu.acceleration;
    // double norm_sq = quat_norm(q); // compute square of the norm
    // norm_sq *= norm_sq;
    a[0] -= 2.0 * (q[1] * q[3] - q[0] * q[2]) /* / norm_sq */;
    a[1] -= 2.0 * (q[0] * q[1] + q[2] * q[3]) /* / norm_sq */;
    a[2] -=  q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3] /* / norm_sq */;
}

// Compute velocity and position with trapezoidal rule integration.
void update_state(struct INS_s *ins)
{
    static uint16_t prev_ms;
    static float prev_range;

    if (ins->imu.time.millisecond == prev_ms)
        return; // data was already processed

    filter_gravity(ins);

    // Integrate acceleration to compute position.
    for (uint8_t i = 0; i < 3; ++i)
    {
        ins->velocity[i] += ins->imu.acceleration[i] * delta(prev_ms, ins->imu.time.millisecond) * INS_SCALE_VELOCITY;
        ins->position[i] += ins->velocity[i] * delta(prev_ms, ins->imu.time.millisecond);
    }

    // us sensor and altimeter fusion.
    if (abs(ins->us_range - prev_range) > INS_RANGE_DER_FILTER && ins->us_range != prev_range)
    {
        ins->position[2] *= INS_ACC_WEIGHT;
        ins->position[2] += ins->us_range * INS_RANGE_WEIGHT;
    }
}
