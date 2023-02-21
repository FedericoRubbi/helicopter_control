#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include <math.h>
#include <stdio.h>

#include "actuator.h"

// Initialize actuator data and configurations.
void init_servo(struct Servo_s *servo, uint8_t pin)
{
    servo->pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    servo->slice = pwm_gpio_to_slice_num(pin);
    servo->config = pwm_get_default_config();

    pwm_config_set_clkdiv(&servo->config, ACT_S_DIV);
    pwm_config_set_wrap(&servo->config, ACT_S_WRAP);
    pwm_init(servo->slice, &servo->config, true);
}

// Initialize actuator data and configurations.
void init_motor(struct Motor_s *motor, uint8_t pin)
{
    motor->pin = pin;
    gpio_set_function(pin, GPIO_FUNC_PWM);
    motor->slice = pwm_gpio_to_slice_num(pin);
    motor->config = pwm_get_default_config();

    pwm_config_set_clkdiv(&motor->config, ACT_M_DIV);
    pwm_config_set_wrap(&motor->config, ACT_M_WRAP);
    pwm_init(motor->slice, &motor->config, true);
}

// Set duty cycle period in milliseconds.
static inline void set_duty_period(struct Servo_s *servo)
{
    pwm_set_gpio_level(servo->pin, (servo->duty_period / ACT_S_PERIOD_MS) * ACT_S_WRAP);
}

// Set duty cycle level as percentage.
static inline void set_duty_value(struct Motor_s *motor)
{
    pwm_set_gpio_level(motor->pin, motor->duty * ACT_M_WRAP / 100.f);
}

// Set servos signals bounds.
static inline void set_servo_bound(struct Servo_s *r_servo, struct Servo_s *l_servo)
{
    float radius = hypot(r_servo->duty_period - ACT_RS_MIDPOINT, l_servo->duty_period - ACT_LS_MIDPOINT);
    if (radius > ACT_S_RADIUS)
    { // set max radius and convert to cartesian coordinates
        float angle = atan2(r_servo->duty_period - ACT_RS_MIDPOINT, l_servo->duty_period - ACT_LS_MIDPOINT);
        r_servo->duty_period = sin(angle) * ACT_S_RADIUS + ACT_RS_MIDPOINT;
        l_servo->duty_period = cos(angle) * ACT_S_RADIUS + ACT_LS_MIDPOINT;
    }
}

// Update servos signals.
void update_servos(struct Servo_s *r_servo, struct Servo_s *l_servo)
{
    set_servo_bound(r_servo, l_servo);
    set_duty_period(r_servo);
    set_duty_period(l_servo);
}

// Update motor drivers signals.
void update_motors(struct Motor_s *l_motor, struct Motor_s *u_motor)
{
    set_duty_value(l_motor);
    set_duty_value(u_motor);
}

void setup_servos(struct Servo_s *r_servo, struct Servo_s *l_servo)
{
    // Initialize servos.
    init_servo(r_servo, ACT_RS_PIN);
    init_servo(l_servo, ACT_LS_PIN);

    // Set starting duty values.
    r_servo->duty_period = ACT_RS_START_MS;
    l_servo->duty_period = ACT_LS_START_MS;
    update_servos(r_servo, l_servo);
}

void setup_motors(struct Motor_s *l_motor, struct Motor_s *u_motor)
{
    // Initialize motor drivers.
    init_motor(l_motor, ACT_LM_PIN);
    init_motor(u_motor, ACT_UM_PIN);

    // Set starting duty values.
    l_motor->duty = 0.0f;
    u_motor->duty = 0.0f;
    update_motors(l_motor, u_motor);
}