#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>

#include "actuator.h"

// Initialize upper and lower servos.
// struct Actuator_s servo[2];

// Set bounds for servos signal.
// Set variable in specified lower and upper bound.
static inline float set_bounds(const float x, const float l_b, const float u_b)
{
    return x < l_b ? l_b : (x > u_b ? u_b : x);
}

// Initialize servo data and configurations.
void init_servo(struct Actuator_s* servo, uint8_t pin)
{
    servo->pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    servo->slice = pwm_gpio_to_slice_num(pin);
    servo->config = pwm_get_default_config();

    pwm_config_set_clkdiv(&servo->config, ACT_S_DIV);
    pwm_config_set_wrap(&servo->config, ACT_S_WRAP);
    pwm_init(servo->slice, &servo->config, true);
}

// Set duty cycle in milliseconds.
void set_millis(struct Actuator_s* servo, float millis)
{
    pwm_set_gpio_level(servo->pin, (millis / ACT_S_PERIOD_MS) * ACT_S_WRAP);
}

// Set duty cycle for right servo.
void set_right_servo(struct Actuator_s* r_servo)
{
    set_millis(r_servo, set_bounds(r_servo->duty, ACT_RS_MIN_MS, ACT_RS_MAX_MS));
}

// Set duty cycle for left servo.
void set_left_servo(struct Actuator_s* l_servo)
{
    set_millis(l_servo, set_bounds(l_servo->duty, ACT_LS_MIN_MS, ACT_LS_MAX_MS));
}

