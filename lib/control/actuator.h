#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <hardware/pwm.h>
#include <stdint.h>

// Motor drivers and servos pin configuration.
#define ACT_RS_PIN 14 // left servo pin
#define ACT_LS_PIN 15 // right servo pin
#define ACT_LM_PIN 28 // lower motor pin
#define ACT_UM_PIN 29 // upper motor pin

// PWM related macros. Processor clock is used.
// For 50Hz servos minimum valid divider is about 39 as wrap (top) is 16 bits.
// Smaller clock divider gives higher resolution.
#define ACT_S_FREQ 50.0f                       // servo update frequency in Hz
#define ACT_S_PERIOD_MS (1000.0f / ACT_S_FREQ) // update period in milliseconds
#define ACT_S_DIV 64.0f
#define ACT_S_WRAP (clock_get_hz(clk_sys) / ACT_S_DIV / ACT_S_FREQ)

// Tuned servos signal bounds for duty cycles.
#define ACT_S_SPAN 0.45f // servo signal span in ms from midpoint
#define ACT_S_RADIUS (ACT_S_SPAN / 2.0f)
#define ACT_RS_MIDPOINT 1.1f                                // right servo midpoint
#define ACT_LS_MIDPOINT 1.45f                               // left servo midpoint
#define ACT_RS_MIN_MS (ACT_RS_MIDPOINT - ACT_S_SPAN / 2.0f) // right servo min duty cycle (down)
#define ACT_RS_MAX_MS (ACT_RS_MIDPOINT + ACT_S_SPAN / 2.0f) // right servo max duty cycle (up)
#define ACT_LS_MIN_MS (ACT_LS_MIDPOINT - ACT_S_SPAN / 2.0f) // left servo min duty cycle (up)
#define ACT_LS_MAX_MS (ACT_LS_MIDPOINT + ACT_S_SPAN / 2.0f) // left servo max duty cycle (down)
#define ACT_RS_START_MS ACT_RS_MIDPOINT
#define ACT_LS_START_MS ACT_LS_MIDPOINT

// Motor drivers.
#define ACT_M_MIDPOINT 1.5f // motor midpoint signal in ms
#define ACT_M_START_MS ACT_M_MIDPOINT
#define ACT_M_MIN_MS ACT_M_MIDPOINT
#define ACT_M_MAX_MS 2.0f

// Pwm-driven actuator struct.
struct Actuator_s
{
    uint8_t pin;
    uint slice;
    pwm_config config;
    float duty; // duty period in ms
};

void init_actuator(struct Actuator_s *servo, uint8_t pin);
void update_servos(struct Actuator_s *r_servo, struct Actuator_s *l_servo);
void update_motors(struct Actuator_s *l_motor, struct Actuator_s *u_motor);
void setup_servos(struct Actuator_s *r_servo, struct Actuator_s *l_servo);
void setup_motors(struct Actuator_s *l_motor, struct Actuator_s *u_motor);

#endif // ACTUATOR_H