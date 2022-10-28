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

static inline void read_raw_time(struct RawData_s *raw_data)
{
    read_register(INS_YYMM, (uint8_t *)&raw_data->time, INS_DATA_LEN);
}

static inline void read_raw_acc(struct RawData_s *raw_data)
{
    read_register(INS_AX, (uint8_t *)&raw_data->acc, INS_DATA_LEN);
    // printf("INS acceleration: %d\t%d\t%d\n", raw_data->acc.a[0], raw_data->acc.a[1],
    //         raw_data->acc.a[2]);
}

static inline void read_raw_gyro(struct RawData_s *raw_data)
{
    read_register(INS_GX, (uint8_t *)&raw_data->gyro, INS_DATA_LEN);
}

static inline void read_raw_mag(struct RawData_s *raw_data)
{
    read_register(INS_HX, (uint8_t *)&raw_data->mag, INS_DATA_LEN);
}

static inline void read_raw_angle(struct RawData_s *raw_data)
{
    read_register(INS_ROLL, (uint8_t *)&raw_data->angles, INS_DATA_LEN);
}

static inline void read_raw_dstatus(struct RawData_s *raw_data)
{
    read_register(INS_D0STATUS, (uint8_t *)&raw_data->dstatus, INS_DATA_LEN);
}

static inline void read_raw_press(struct RawData_s *raw_data)
{
    read_register(INS_PRESSUREL, (uint8_t *)&raw_data->press, INS_DATA_LEN);
}

static inline void read_raw_lonlat(struct RawData_s *raw_data)
{
    read_register(INS_LONL, (uint8_t *)&raw_data->lonlat, INS_DATA_LEN);
}

static inline void read_raw_gps(struct RawData_s *raw_data)
{
    read_register(INS_GPSHEIGHT, (uint8_t *)&raw_data->gpsv, INS_DATA_LEN);
}

static inline void read_raw_quat(struct RawData_s *raw_data)
{
    read_register(INS_Q0, (uint8_t *)&raw_data->quat, INS_DATA_LEN);
}

// Read specified data and write it to INS_s struct.
void read_data(struct INS_s *ins)
{
    // Read and scale all data.
#ifdef INS_READ_TIME
    read_raw_time(&ins->raw_data);
    ins->time = ins->raw_data.time;
#endif
#ifdef INS_READ_ACC
    read_raw_acc(&ins->raw_data);
    for (uint8_t i = 0; i < 3; ++i)
        ins->acceleration[i] = (float)(ins->raw_data.acc.a[i] * INS_SCALE_ACC);
#endif
#ifdef INS_READ_GYRO
    read_raw_gyro(&ins->raw_data);
    for (uint8_t i = 0; i < 3; ++i)
        ins->angular_velocity[i] = (float)ins->raw_data.gyro.w[i] * INS_SCALE_GYRO;
#endif
#ifdef INS_READ_MAG
    read_raw_mag(&ins->raw_data);
    for (uint8_t i = 0; i < 3; ++i)
        ins->magnetic_field[i] = ins->raw_data.mag.h[i];
#endif
#ifdef INS_READ_ANGLE
    read_raw_angle(&ins->raw_data);
    for (uint8_t i = 0; i < 3; ++i)
        ins->angle[i] = (float)ins->raw_data.angles.angle[i] * INS_SCALE_ANGLE;
#endif
#ifdef INS_READ_DSTATUS
    read_raw_dstatus(&ins->raw_data);
    for (uint8_t i = 0; i < 3; ++i)
        ins->dstatus[i] = ins->raw_data.dstatus.DStatus[i];
#endif
#ifdef INS_READ_PRESS
    read_raw_press(&ins->raw_data);
    ins->pressure = ins->raw_data.press.pressure;
    ins->altitude = (float)ins->raw_data.press.altitude * INS_SCALE_ALTITUDE;
#endif
#ifdef INS_READ_LONLAT
    read_raw_lonlat(&ins->raw_data);
    ins->latitude = ins->raw_data.lonlat.lat;
    ins->longitude = ins->raw_data.lonlat.lon;
#endif
#ifdef INS_READ_GPS
    read_raw_gps(&ins->raw_data);
    ins->gps_velocity = (float)ins->raw_data.gpsv.GPSVelocity * INS_SCALE_GPSV;
    ins->gps_yaw = (float)ins->raw_data.gpsv.GPSYaw * INS_SCALE_GPSY;
    ins->gps_height = (float)ins->raw_data.gpsv.GPSHeight * INS_SCALE_GPSH;
#endif
#ifdef INS_READ_QUAT
    read_raw_quat(&ins->raw_data);
    for (uint8_t i = 0; i < 3; ++i)
        ins->quaternion[i] = (float)ins->raw_data.quat.q[i] * INS_SCALE_QUAT;
#endif
        // Update temperature depending on read data.
#ifdef INS_READ_ACC
    ins->temperature = (float)ins->raw_data.angles.t * INS_SCALE_TEMP;
#elif defined(INS_READ_GYRO)
    ins->temperature = (float)ins->raw_data.gyro.t * INS_SCALE_TEMP;
#elif defined(INS_READ_MAG)
    ins->temperature = (float)ins->raw_data.mag.t * INS_SCALE_TEMP;
#elif defined(INS_READ_ANGLE)
    ins->temperature = (float)ins->raw_data.angles.t * INS_SCALE_TEMP;
#endif
}
