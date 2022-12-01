#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

#include "actuator.h"

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

// Set duty cycle level.
static inline void set_duty(struct Actuator_s* servo)
{
    pwm_set_gpio_level(servo->pin, (servo->duty / ACT_S_PERIOD_MS) * ACT_S_WRAP);
}

// Set servos signals bounds.
static inline void set_servo_bound(struct Actuator_s* r_servo, struct Actuator_s* l_servo)
{
    float radius = hypot(r_servo->duty - ACT_RS_MIDPOINT, l_servo->duty - ACT_LS_MIDPOINT);
    if (radius > ACT_S_RADIUS) { // set max radius and convert to cartesian coordinates
        float angle = atan2(r_servo->duty - ACT_RS_MIDPOINT, l_servo->duty - ACT_LS_MIDPOINT);
        r_servo->duty = sin(angle) * ACT_S_RADIUS + ACT_RS_MIDPOINT;
        l_servo->duty = cos(angle) * ACT_S_RADIUS + ACT_LS_MIDPOINT;
    }
}

// Update servos signals.
void update_servos(struct Actuator_s* r_servo, struct Actuator_s* l_servo)
{
    set_servo_bound(r_servo, l_servo);
    set_duty(r_servo);
    set_duty(l_servo);
}
