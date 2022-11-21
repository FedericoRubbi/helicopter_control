#ifndef US_SENSOR_H
#define US_SENSOR_H

#include <stdint.h>

// Ultrasonic sensor configuration.
#define US_DEFAULT_ADDR 0x70 // default address
#define US_SET_KALMAN // comment to disable Kalman filter

#define US_RANGE 0x51 // range command
#define US_CHANGE_ADDR0 0xAA // send in order US_CHANGE_ADDR0 and US_CHANGE_ADDR1 to change address
#define US_CHANGE_ADDR1 0xA5
#define US_MAX_GRADIENT 5 // max range derivative in absolute value to apply kalman filter
#define US_MAX_SAMPLE_TIME 0.067
#define US_SCALE_RANGE (1.0 / 100.0) // convert range measure to meters

#ifdef US_SET_KALMAN // Kalman filter coefficients
#define US_KALMAN_R 0.008f
#define US_KALMAN_Q 0.084535f
#endif

float read_range();

#endif // US_SENSOR_H