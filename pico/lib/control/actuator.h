#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <hardware/pwm.h>
#include <stdint.h>

// Motor drivers and servos pin configuration.
#define ACT_RS_PIN 14 // left servo pin
#define ACT_LS_PIN 15 // right servo pin
#define ACT_LM_PIN 26 // lower motor pin
#define ACT_UM_PIN 27 // upper motor pin

// PWM related macros. Processor clock is used.
// For 50Hz servos minimum valid divider is about 39 as wrap (top) is 16 bits.
// Smaller clock divider gives higher resolution.
#define ACT_S_FREQ 50.0f                       // servo update frequency in Hz
#define ACT_S_PERIOD_MS (1000.0f / ACT_S_FREQ) // update period in milliseconds
#define ACT_S_DIV 64.0f
#define ACT_S_WRAP (clock_get_hz(clk_sys) / ACT_S_DIV / ACT_S_FREQ)

#define ACT_M_FREQ 50000.0f                    // motor driver update frequency in Hz
#define ACT_M_PERIOD_MS (1000.0f / ACT_M_FREQ) // update period in milliseconds
#define ACT_M_DIV 1.0f
#define ACT_M_WRAP (clock_get_hz(clk_sys) / ACT_M_DIV / ACT_M_FREQ)

// Tuned servos signal bounds for duty cycles.
#define ACT_S_SPAN 0.45f // servo signal span in ms from midpoint
#define ACT_S_RADIUS (ACT_S_SPAN / 2.0f)
#define ACT_RS_MIDPOINT 1.1f                                // right servo midpoint
#define ACT_LS_MIDPOINT 1.45f                               // left servo midpoint
#define ACT_RS_MIN_MS (ACT_RS_MIDPOINT - ACT_S_RADIUS) // right servo min duty cycle (down)
#define ACT_RS_MAX_MS (ACT_RS_MIDPOINT + ACT_S_RADIUS) // right servo max duty cycle (up)
#define ACT_LS_MIN_MS (ACT_LS_MIDPOINT - ACT_S_RADIUS) // left servo min duty cycle (up)
#define ACT_LS_MAX_MS (ACT_LS_MIDPOINT + ACT_S_RADIUS) // left servo max duty cycle (down)
#define ACT_RS_START_MS ACT_RS_MIDPOINT
#define ACT_LS_START_MS ACT_LS_MIDPOINT

// TODO: update with new driver values!
// Motor drivers.
#define ACT_M_MIN 0.0f   // min duty value
#define ACT_M_MAX 100.0f // max duty value
#define ACT_M_STOP 0.0f
#define ACT_M_START ACT_M_STOP // starting duty value
#define ACT_M_TAKEOFF 60.0f    // to be tuned

// Pwm-driven servo struct.
struct Servo_s
{
    uint8_t pin;
    uint32_t slice;
    pwm_config config;
    float duty_period; // duty period in ms
};

// Pwm-driven motor struct.
struct Motor_s
{
    uint8_t pin;
    uint32_t slice;
    pwm_config config;
    float duty; // duty percentage value
};

void init_servo(struct Servo_s *servo, uint8_t pin);
void init_motor(struct Motor_s *motor, uint8_t pin);
void update_servos(struct Servo_s *r_servo, struct Servo_s *l_servo);
void update_motors(struct Motor_s *l_motor, struct Motor_s *u_motor);
void setup_servos(struct Servo_s *r_servo, struct Servo_s *l_servo);
void setup_motors(struct Motor_s *l_motor, struct Motor_s *u_motor);

#endif // ACTUATOR_H