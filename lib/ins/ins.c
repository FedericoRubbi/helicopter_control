#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "ins.h"

// Initialize static data.
// static struct RawData_s raw_data;

// Write register address to read and read resonse bytes from INS address
static inline void read_register(uint8_t addr, uint8_t buffer[],
                                 uint8_t buffer_size)
{
    // Write address of register to read.
    i2c_write_blocking(i2c1, INS_DEFAULT_ADDR, &addr, 1, true); // nostop set to keep control of the bus
    // Fill buffer with data.
    i2c_read_blocking(i2c1, INS_DEFAULT_ADDR, buffer, buffer_size, false); // nostop unset to release bus
}

// Scale data to standard units.
void scale_data(struct INS_s *ins)
{
    // Copy time, magnetic field, pin inputs, pressure, latitude and longitude
    // and scale remaining data.
    ins->time = ins->raw_data.time;

    for (uint8_t i = 0; i < 3; ++i)
        ins->acceleration[i] = (float)(ins->raw_data.acc[i] * INS_SCALE_ACC);
    for (uint8_t i = 0; i < 3; ++i)
        ins->angular_velocity[i] = (float)ins->raw_data.gyro[i] * INS_SCALE_GYRO;
    for (uint8_t i = 0; i < 3; ++i)
        ins->magnetic_field[i] = ins->raw_data.mag[i];
    for (uint8_t i = 0; i < 3; ++i)
        ins->angle[i] = (float)ins->raw_data.angle[i] * INS_SCALE_ANGLE;

    ins->temperature = (float)ins->raw_data.temperature * INS_SCALE_TEMP;

    for (uint8_t i = 0; i < 3; ++i)
        ins->dstatus[i] = ins->raw_data.dstatus[i];

    ins->pressure = ins->raw_data.pressure;
    ins->altitude = (float)ins->raw_data.altitude * INS_SCALE_ALTITUDE;

    ins->latitude = ins->raw_data.latitude;
    ins->longitude = ins->raw_data.longitude;

    ins->gps_velocity = (float)ins->raw_data.gps_velocity * INS_SCALE_GPSV;
    ins->gps_yaw = (float)ins->raw_data.gps_yaw * INS_SCALE_GPSY;
    ins->gps_height = (float)ins->raw_data.gps_height * INS_SCALE_GPSH;

    for (uint8_t i = 0; i < 4; ++i)
        ins->quaternion[i] = (float)ins->raw_data.quat[i] * INS_SCALE_QUAT;
}

// Read specified data and write it to INS_s struct.
void read_data(struct INS_s *ins)
{
    // Read all data registers.
    read_register(INS_YYMM, (uint8_t *)&ins->raw_data, sizeof(ins->raw_data));

    scale_data(ins);
}
