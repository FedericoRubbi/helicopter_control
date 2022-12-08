#ifndef INS_H
#define INS_H

#include <stdint.h>

#include "imu.h"
#include "us_sensor.h"

// INS configurations.
#define INS_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN // pin 6 for waveshare board
#define INS_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN // pin 7 for waveshare board
#define INS_MAX_BAUDRATE 400000              // Baudrate in Hz
#define INS_DEFAULT_BAUDRATE INS_MAX_BAUDRATE
#define INS_GRAVITY_ACC (9.81)
#define INS_RANGE_DER_FILTER 0.2 // us sensor range maximum derivative in meters
#define INS_ACC_WEIGHT  0.5 // accelerometer height estimate weight
#define INS_RANGE_WEIGHT 0.5 // us sensor range estimate weight

#define INS_PACKET_SIZE (sizeof(struct RawData_s)) // packet size macro for transmission
#define INS_SAMPLE_TIME IMU_SAMPLE_TIME

#define INS_SCALE_VELOCITY INS_GRAVITY_ACC // to convert g to m/s² when integrating acceleration

struct INS_s
{
    struct IMU_s imu;
    float us_range; // ultrasonic sensor range
    double position[3];
    double velocity[3];
};

void setup_ins(struct INS_s *ins);
static inline void filter_gravity(struct INS_s *ins);
void update_state(struct INS_s *); // compute velocity and position

#endif // INS_H