#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <hardware/pwm.h>
#include <stdint.h>

// Servos pin configuration.
#define ACT_RS_PIN 14 // left servo pin
#define ACT_LS_PIN 15 // right servo pin

// PWM related macros. Processor clock is used.
// For 50Hz servos minimum valid divider is about 39 as wrap (top) is 16 bits.
// Smaller clock divider gives higher resolution.
#define ACT_S_FREQ 50.0f // servo update frequency in Hz
#define ACT_S_PERIOD_MS (1000.0f / ACT_S_FREQ) // update period in milliseconds
#define ACT_S_DIV 64.0f
#define ACT_S_WRAP (clock_get_hz(clk_sys) / ACT_S_DIV / ACT_S_FREQ)

// TODO: power servos from BEC and check bound values.
// Tuned signal bounds.
#define ACT_RS_START_MS 0
#define ACT_RS_MIN_MS 0.6f // min duty cycle in ms for left servo (downwards)
#define ACT_RS_MAX_MS 1.5f // max duty cycle in ms for left servo (upwards)
#define ACT_LS_START_MS 0
#define ACT_LS_MIN_MS 1.0f // min duty cycle in ms for right servo (upwards)
#define ACT_LS_MAX_MS 1.9f // max duty cycle in ms for right servo (downwards)

struct Actuator_s {
    uint8_t pin;
    uint slice;
    pwm_config config;
    uint16_t duty; // duty period in ms
};

void init_servo(struct Actuator_s* servo, uint8_t pin);
void set_millis(struct Actuator_s* servo, float millis);
void set_right_servo(struct Actuator_s* r_servo);
void set_left_servo(struct Actuator_s* l_servo);

#endif // ACTUATOR_H