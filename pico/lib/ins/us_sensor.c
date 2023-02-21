#include <stdio.h>
#include "hardware/i2c.h"
#include "pico/stdlib.h"

#include "us_sensor.h"

#ifdef US_SET_KALMAN
static inline int abs(int x) { return x >= 0 ? x : -x; }

// Apply fast Kalman filter to range measure.
void kalman_filter(float *range)
{
    const float R = US_KALMAN_R;
    const float Q = US_KALMAN_Q;
    static int prev_range[2]; // save previous ranges to compute derivative
    static float mu;
    static float sigma = US_KALMAN_R;

    // Check gradient.
    if (abs(prev_range[0] - *range) <= US_MAX_GRADIENT || abs(prev_range[0] - prev_range[1]) > US_MAX_GRADIENT)
    {
        mu += sigma / (sigma + Q) * (*range - mu);
        sigma = sigma * Q / (sigma + Q) + R;
    }
    prev_range[1] = prev_range[0]; // update ranges
    prev_range[0] = *range;

    *range = mu; // corrected distance
}
#endif

// TODO: change waiting system for variable frequency reading using i2c_read_timeout_us.
// Read blocking range. Measurement can take up to 66.7ms at full range.
void read_range(float *range)
{
    static uint8_t buf[2];
    const uint8_t cmd = US_RANGE;
    i2c_write_blocking(i2c1, US_DEFAULT_ADDR, &cmd, 1, false);
    i2c_read_blocking(i2c1, US_DEFAULT_ADDR, buf, sizeof(buf), false);
    *range = (float)(buf[0] << 8 | buf[1]); // invert endianness
#ifdef US_SET_KALMAN
    kalman_filter(range);
#endif
    *range *= US_SCALE_RANGE;
}