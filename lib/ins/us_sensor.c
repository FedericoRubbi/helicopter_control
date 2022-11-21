#include "hardware/i2c.h"

#include "us_sensor.h"
#include <stdio.h>

#ifdef US_SET_KALMAN
struct USKalman_s kf = {
    .A = 1,
    .B = 0,
    .C = 1,
    .u = 0,
    .R = 0.008f,
    .Q = 0.084535f,
};
#endif

static inline int abs(int x) { return x >= 0 ? x : -x; }

#ifdef US_SET_KALMAN
// Apply Kalman filter to range measure.
float kalman_filter(float range)
{
    static int prev_range[2]; // save previous ranges to compute derivative
    static float prev_mu;
    static float prev_sigma;

    // Check gradient.
    if (abs(prev_range[0] - range) <= US_MAX_GRADIENT
        || abs(prev_range[0] - prev_range[1]) > US_MAX_GRADIENT) {
        float mu_bar = kf.A * prev_mu;
        float sigma_bar = kf.A * prev_sigma * kf.A + kf.R;
        float K = sigma_bar * kf.C / (kf.C * sigma_bar * kf.C + kf.Q);
        float mu = mu_bar + K * (range - kf.C * mu_bar);
        float sigma = (1 - K * kf.C) * sigma_bar;

        prev_mu = mu;
        prev_sigma = sigma;
    }
    prev_range[1] = prev_range[0]; // update ranges
    prev_range[0] = range;

    return prev_mu; // corrected distance
}
#endif

// TODO: change waiting system for variable frequency reading.
float read_range()
{
    static int16_t range = 0;
    static uint8_t buf[2];
    const uint8_t cmd = US_RANGE;
    i2c_write_blocking(i2c1, US_DEFAULT_ADDR, &cmd, 1, false);
    sleep_ms(100);
    i2c_read_blocking(i2c1, US_DEFAULT_ADDR, (uint8_t*)buf, sizeof(buf), false);
    range = buf[0] << 8 | buf[1]; // invert endianness
#ifdef US_SET_KALMAN
    return kalman_filter((float)range) * US_SCALE_RANGE;
#elif
    return (float)range * US_SCALE_RANGE;
#endif
}