#include "hardware/i2c.h"

#include "imu.h"
#include <stdlib.h>

// Write register address to read and read resonse bytes from IMU address
static inline void read_register(uint8_t addr, uint8_t buffer[], uint8_t buffer_size)
{
    // Write address of register to read.
    i2c_write_blocking(i2c1, IMU_DEFAULT_ADDR, &addr, 1,
        true); // nostop set to keep control of the bus
    // Fill buffer with data.
    i2c_read_blocking(i2c1, IMU_DEFAULT_ADDR, buffer, buffer_size,
        false); // nostop unset to release bus
}

// Copy scaled data in standard units to IMU struct.
void format_imu_data(struct IMU_s* imu)
{
#ifdef IMU_READ_TIME
    imu->time = imu->raw_data.time;
#endif
#ifdef IMU_READ_ACC
    for (uint8_t i = 0; i < 3; ++i)
        imu->acceleration[i] = (double)imu->raw_data.acc[i] * IMU_SCALE_ACC;
#endif
#ifdef IMU_READ_GYRO
    for (uint8_t i = 0; i < 3; ++i)
        imu->angular_velocity[i] = (double)imu->raw_data.gyro[i] * IMU_SCALE_GYRO;
#endif
#ifdef IMU_READ_MAG
    for (uint8_t i = 0; i < 3; ++i)
        imu->magnetic_field[i] = imu->raw_data.mag[i];
#endif
#ifdef IMU_READ_ANGLE
    for (uint8_t i = 0; i < 3; ++i)
        imu->angle[i] = (float)imu->raw_data.angle[i] * IMU_SCALE_ANGLE;
#endif
#ifdef IMU_READ_TEMP
    imu->temperature = (float)imu->raw_data.temperature * IMU_SCALE_TEMP;
#endif
#ifdef IMU_READ_DSTATUS
    for (uint8_t i = 0; i < 4; ++i)
        imu->dstatus[i] = imu->raw_data.dstatus[i]; // * IMU_SCALE_DSTATUS for analog input;
#endif
#ifdef IMU_READ_PRESS
    imu->pressure = imu->raw_data.pressure;
#endif
#ifdef IMU_READ_ALTITUDE
    imu->altitude = (float)imu->raw_data.altitude * IMU_SCALE_ALTITUDE;
#endif
#ifdef IMU_READ_LONLAT
    imu->latitude = imu->raw_data.latitude;
    imu->longitude = imu->raw_data.longitude;
#endif
#ifdef IMU_READ_GPS
    imu->gps_velocity = (float)imu->raw_data.gps_velocity * IMU_SCALE_GPSV;
    imu->gps_yaw = (float)imu->raw_data.gps_yaw * IMU_SCALE_GPSY;
    imu->gps_height = (float)imu->raw_data.gps_height * IMU_SCALE_GPSH;
#endif
#ifdef IMU_READ_QUAT
    for (uint8_t i = 0; i < 4; ++i)
        imu->quaternion[i] = imu->raw_data.quat[i] * IMU_SCALE_QUAT;

#endif
}

// Read all data from IMU. All read macros must be enabled and RawData_s must be packed to use.
// The function is almost 2 times faster than read_imu_data when all read flags are set.
void read_imu_all(struct IMU_s* imu) {
    read_register(IMU_YYMM, (uint8_t*)&imu->raw_data, sizeof(imu->raw_data));
}

// Read only specified data and write it to RawData_s.
// This function can be faster than read_imu_all when some read flags are disabled.
void read_imu_data(struct IMU_s* imu)
{
#ifdef IMU_READ_TIME
    read_register(IMU_YYMM, (uint8_t*)&imu->raw_data.time, sizeof(imu->raw_data.time));
#endif
#ifdef IMU_READ_ACC
    read_register(IMU_AX, (uint8_t*)imu->raw_data.acc, sizeof(imu->raw_data.acc));
#endif
#ifdef IMU_READ_GYRO
    read_register(IMU_GX, (uint8_t*)imu->raw_data.gyro, sizeof(imu->raw_data.gyro));
#endif
#ifdef IMU_READ_MAG
    read_register(IMU_HX, (uint8_t*)imu->raw_data.mag, sizeof(imu->raw_data.mag));
#endif
#ifdef IMU_READ_ANGLE
    read_register(IMU_ROLL, (uint8_t*)imu->raw_data.angle, sizeof(imu->raw_data.angle));
#endif
#ifdef IMU_READ_TEMP
    read_register(
        IMU_TEMP, (uint8_t*)&imu->raw_data.temperature, sizeof(imu->raw_data.temperature));
#endif
#ifdef IMU_READ_DSTATUS
    read_register(IMU_D0STATUS, (uint8_t*)imu->raw_data.dstatus, sizeof(imu->raw_data.dstatus));
#endif
#ifdef IMU_READ_PRESS
    read_register(IMU_PRESSUREL, (uint8_t*)&imu->raw_data.pressure, sizeof(imu->raw_data.pressure));
#endif
#ifdef IMU_READ_ALTITUDE
    read_register(IMU_HEIGHTL, (uint8_t*)&imu->raw_data.altitude, sizeof(imu->raw_data.altitude));
#endif
#ifdef IMU_READ_LONLAT
    read_register(IMU_LONL, (uint8_t*)&imu->raw_data.latitude, 2 * sizeof(imu->raw_data.latitude));
#endif
#ifdef IMU_READ_GPS
    read_register(IMU_GPSHEIGHT, (uint8_t*)&imu->raw_data.gps_height,
        sizeof(imu->raw_data.gps_height) + sizeof(imu->raw_data.gps_yaw)
            + sizeof(imu->raw_data.gps_velocity));
#endif
#ifdef IMU_READ_QUAT
    read_register(IMU_Q0, (uint8_t*)imu->raw_data.quat, sizeof(imu->raw_data.quat));
#endif
}