#ifndef INS_H
#define INS_H

#include <stdint.h>

#include "imu.h"

// INS configurations.
#define INS_MAX_BAUDRATE 400000 // Baudrate in Hz
#define INS_DEFAULT_BAUDRATE INS_MAX_BAUDRATE
#define INS_GRAVITY_ACC (9.81)

#define INS_PACKET_SIZE (sizeof(struct RawData_s)) // packet size macro for transmission

#define INS_SCALE_VELOCITY INS_GRAVITY_ACC // to convert g to m/s² when integrating acceleration

struct INS_s {
    struct IMU_s imu;
    double position[3];
    double velocity[3];

};

static inline void read_register(uint8_t, uint8_t[], uint8_t);
void format_data(struct INS_s*);
void read_data(struct INS_s*);
void update_state(struct INS_s*); // compute velocity and position

#endif // INS_H