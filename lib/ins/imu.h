#ifndef IMU_H
#define IMU_H

#include <stdint.h>

// IMU configurations.
#define IMU_DEFAULT_ADDR 0x50 // keep default address
#define IMU_SAMPLE_TIME 0.005 // Minimum sampling time in seconds

// Read options for small optimizations. Uncomment to select.
#define IMU_READ_TIME
#define IMU_READ_ACC
#define IMU_READ_GYRO
// #define IMU_READ_MAG // not needed
#define IMU_READ_ANGLE
// #define IMU_READ_TEMP // not needed
// #define IMU_READ_DSTATUS // not needed
// #define IMU_READ_PRESS // not needed
#define IMU_READ_ALTITUDE
// #define IMU_READ_LONLAT // not needed
// #define IMU_READ_GPS // not needed
#define IMU_READ_QUAT

// Command registers address.
#define IMU_SAVE 0x00
#define IMU_CALSW 0x01
#define IMU_RSW 0x02
#define IMU_RRATE 0x03
#define IMU_BAUD 0x04
#define IMU_AXOFFSET 0x05
#define IMU_AYOFFSET 0x06
#define IMU_AZOFFSET 0x07
#define IMU_GXOFFSET 0x08
#define IMU_GYOFFSET 0x09
#define IMU_GZOFFSET 0x0a
#define IMU_HXOFFSET 0x0b
#define IMU_HYOFFSET 0x0c
#define IMU_HZOFFSET 0x0d
#define IMU_D0MODE 0x0e
#define IMU_D1MODE 0x0f
#define IMU_D2MODE 0x10
#define IMU_D3MODE 0x11
#define IMU_D0PWMH 0x12
#define IMU_D1PWMH 0x13
#define IMU_D2PWMH 0x14
#define IMU_D3PWMH 0x15
#define IMU_D0PWMT 0x16
#define IMU_D1PWMT 0x17
#define IMU_D2PWMT 0x18
#define IMU_D3PWMT 0x19
#define IMU_IICADDR 0x1a
#define IMU_LEDOFF 0x1b
#define IMU_GPSBAUD 0x1c

// Data register address.
#define IMU_YYMM 0x30
#define IMU_DDHH 0x31
#define IMU_MMSS 0x32
#define IMU_MS 0x33
#define IMU_AX 0x34
#define IMU_AY 0x35
#define IMU_AZ 0x36
#define IMU_GX 0x37
#define IMU_GY 0x38
#define IMU_GZ 0x39
#define IMU_HX 0x3a
#define IMU_HY 0x3b
#define IMU_HZ 0x3c
#define IMU_ROLL 0x3d
#define IMU_PITCH 0x3e
#define IMU_YAW 0x3f
#define IMU_TEMP 0x40
#define IMU_D0STATUS 0x41
#define IMU_D1STATUS 0x42
#define IMU_D2STATUS 0x43
#define IMU_D3STATUS 0x44
#define IMU_PRESSUREL 0x45
#define IMU_PRESSUREH 0x46
#define IMU_HEIGHTL 0x47
#define IMU_HEIGHTH 0x48
#define IMU_LONL 0x49
#define IMU_LONH 0x4a
#define IMU_LATL 0x4b
#define IMU_LATH 0x4c
#define IMU_GPSHEIGHT 0x4d
#define IMU_GPSYAW 0x4e
#define IMU_GPSVL 0x4f
#define IMU_GPSVH 0x50
#define IMU_Q0 0x51
#define IMU_Q1 0x52
#define IMU_Q2 0x53
#define IMU_Q3 0x54

// Pin modes.
#define IMU_DIO_MODE_AIN 0
#define IMU_DIO_MODE_DIN 1
#define IMU_DIO_MODE_DOH 2
#define IMU_DIO_MODE_DOL 3
#define IMU_DIO_MODE_DOPWM 4
#define IMU_DIO_MODE_GPS 5

// Data macros and structs.
#define IMU_SCALE_TEMP (1.0f / 100.0f)
#define IMU_SCALE_ACC (16.0 / 32768.0)
#define IMU_SCALE_GYRO (2000.0 / 32768.0)
#define IMU_SCALE_ANGLE (180.0f / 32768.0f)
#define IMU_SCALE_DSTATUS (3.3 / 1024.0) // for analog input
#define IMU_SCALE_ALTITUDE (1.0f / 100.0f)
#define IMU_SCALE_GPSH (1.0f / 10.0f)
#define IMU_SCALE_GPSY (1.0f / 10.0f)
#define IMU_SCALE_GPSV (1.0f / 1000.0f)
#define IMU_SCALE_QUAT (1.0 / 32768.0)

#ifdef IMU_READ_TIME
struct Time_s {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
};
#endif

struct RawData_s {
#ifdef IMU_READ_TIME
    struct Time_s time;
#endif
#ifdef IMU_READ_ACC
    int16_t acc[3];
#endif
#ifdef IMU_READ_GYRO
    int16_t gyro[3];
#endif
#ifdef IMU_READ_MAG
    int16_t mag[3];
#endif
#ifdef IMU_READ_ANGLE
    int16_t angle[3];
#endif
#ifdef IMU_READ_TEMP
    int16_t temperature;
#endif
#ifdef IMU_READ_DSTATUS
    int16_t dstatus[4];
#endif
#ifdef IMU_READ_PRESS
    uint32_t pressure;
#endif
#ifdef IMU_READ_ALTITUDE
    int32_t altitude;
#endif
#ifdef IMU_READ_LONLAT
    int32_t longitude;
    int32_t latitude;
#endif
#ifdef IMU_READ_GPS
    int16_t gps_height;
    int16_t gps_yaw;
    uint32_t gps_velocity;
#endif
#ifdef IMU_READ_QUAT
    int16_t quat[4]; // real part first, imaginary part after
#endif
} __attribute__((packed)); // explicity disable struct padding as required from imu_read_all

struct IMU_s {
    struct RawData_s raw_data;
#ifdef IMU_READ_TIME
    struct Time_s time;
#endif
#ifdef IMU_READ_ACC
    double acceleration[3];
#endif
#ifdef IMU_READ_GYRO
    double angular_velocity[3];
#endif
#ifdef IMU_READ_MAG
    int16_t magnetic_field[3];
#endif
#ifdef IMU_READ_ANGLE
    float angle[3];
#endif
#ifdef IMU_READ_TEMP
    float temperature;
#endif
#ifdef IMU_READ_DSTATUS
    int16_t dstatus[4];
#endif
#ifdef IMU_READ_PRESS
    uint32_t pressure;
#endif
#ifdef IMU_READ_ALTITUDE
    float altitude;
#endif
#ifdef IMU_READ_LONLAT
    int32_t longitude;
    int32_t latitude;
#endif
#ifdef IMU_READ_GPS
    float gps_height;
    float gps_yaw;
    float gps_velocity;
#endif
#ifdef IMU_READ_QUAT
    double quaternion[4];
#endif
};

static inline void read_register(uint8_t, uint8_t[], uint8_t);
void format_imu_data(struct IMU_s*);
void read_imu_data(struct IMU_s*);

#endif // IMU_H