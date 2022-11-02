#include "hardware/i2c.h"

#include "ins.h"

// Write register address to read and read resonse bytes from INS address
static inline void read_register(uint8_t addr, uint8_t buffer[], uint8_t buffer_size)
{
    // Write address of register to read.
    i2c_write_blocking(i2c1, INS_DEFAULT_ADDR, &addr, 1,
        true); // nostop set to keep control of the bus
    // Fill buffer with data.
    i2c_read_blocking(i2c1, INS_DEFAULT_ADDR, buffer, buffer_size,
        false); // nostop unset to release bus
}

// COpy scaled data in standard units to INS struct.
void format_data(struct INS_s* ins)
{
#ifdef INS_READ_TIME
    ins->time = ins->raw_data.time;
#endif
#ifdef INS_READ_ACC
    for (uint8_t i = 0; i < 3; ++i)
        ins->acceleration[i] = (float)(ins->raw_data.acc[i] * INS_SCALE_ACC);
#endif
#ifdef INS_READ_GYRO
    for (uint8_t i = 0; i < 3; ++i)
        ins->angular_velocity[i] = (float)ins->raw_data.gyro[i] * INS_SCALE_GYRO;
#endif
#ifdef INS_READ_MAG
    for (uint8_t i = 0; i < 3; ++i)
        ins->magnetic_field[i] = ins->raw_data.mag[i];
#endif
#ifdef INS_READ_ANGLE
    for (uint8_t i = 0; i < 3; ++i)
        ins->angle[i] = (float)ins->raw_data.angle[i] * INS_SCALE_ANGLE;
#endif
#ifdef INS_READ_TEMP
    ins->temperature = (float)ins->raw_data.temperature * INS_SCALE_TEMP;
#endif
#ifdef INS_READ_DSTATUS
    for (uint8_t i = 0; i < 3; ++i)
        ins->dstatus[i] = ins->raw_data.dstatus[i]; // * INS_SCALE_DSTATUS for analog input;
#endif
#ifdef INS_READ_PRESS
    ins->pressure = ins->raw_data.pressure;
#endif
#ifdef INS_READ_ALTITUDE
    ins->altitude = (float)ins->raw_data.altitude * INS_SCALE_ALTITUDE;
#endif
#ifdef INS_READ_LATLON
    ins->latitude = ins->raw_data.latitude;
    ins->longitude = ins->raw_data.longitude;
#endif
#ifdef INS_READ_GPS
    ins->gps_velocity = (float)ins->raw_data.gps_velocity * INS_SCALE_GPSV;
    ins->gps_yaw = (float)ins->raw_data.gps_yaw * INS_SCALE_GPSY;
    ins->gps_height = (float)ins->raw_data.gps_height * INS_SCALE_GPSH;
#endif
#ifdef INS_READ_QUAT
    for (uint8_t i = 0; i < 4; ++i)
        ins->quaternion[i] = (float)ins->raw_data.quat[i] * INS_SCALE_QUAT;
#endif
}

// Read all data and write it to INS_s struct.
void read_data(struct INS_s* ins)
{
    read_register(INS_YYMM, (uint8_t*)&ins->raw_data, sizeof(ins->raw_data));
}
