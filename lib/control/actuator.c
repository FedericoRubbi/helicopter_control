#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

#include "actuator.h"

// Initialize actuator data and configurations.
void init_actuator(struct Actuator_s* act, uint8_t pin)
{
    act->pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    act->slice = pwm_gpio_to_slice_num(pin);
    act->config = pwm_get_default_config();

    pwm_config_set_clkdiv(&act->config, ACT_S_DIV);
    pwm_config_set_wrap(&act->config, ACT_S_WRAP);
    pwm_init(act->slice, &act->config, true);
}

// Set duty cycle level.
static inline void set_duty(struct Actuator_s* act)
{
    pwm_set_gpio_level(act->pin, (act->duty / ACT_S_PERIOD_MS) * ACT_S_WRAP);
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

// Update motor drivers signals.
void update_motors(struct Actuator_s* l_motor, struct Actuator_s* u_motor)
{
    set_duty(l_motor);
    set_duty(u_motor);
}

void update_servos(struct Actuator_s* r_servo, struct Actuator_s* l_servo)
{
    // Initialize servos.
    init_actuator(r_servo, ACT_RS_PIN);
    init_actuator(l_servo, ACT_LS_PIN);

    // Set starting duty values.
    r_servo.duty = ACT_RS_START_MS;
    l_servo.duty = ACT_LS_START_MS;
    update_servos(r_servo, l_servo);
}

void setup_actuators(struct Actuator_s* l_motor, struct Actuator_s* u_motor)
{
    // Initialize motor drivers.
    init_actuator(l_motor, ACT_LM_PIN);
    init_actuator(u_motor, ACT_UM_PIN);

    // Set starting duty values.
    l_motor.duty = 0.0f;
    u_motor.duty = 0.0f;
    update_motors(&l_motor, &u_motor);
    sleep_ms(ACT_S_PERIOD_MS);
    l_motor.duty = ACT_M_START_MS;
    u_motor.duty = ACT_M_START_MS;
    update_motors(&l_motor, &u_motor);
}