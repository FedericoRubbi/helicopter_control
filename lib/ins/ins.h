#ifndef INS_H
#define INS_H

#include <stdint.h>

// INS configurations.
#define INS_DEFAULT_ADDR 0x50 // keep default address
#define INS_DEFAULT_BAUDRATE 115200
// Uncomment flags to select data to read.
#define INS_READ_TIME
#define INS_READ_ACC
#define INS_READ_GYRO
//#define INS_READ_MAG
#define INS_READ_ANGLE
//#define INS_READ_DSTATUS
#define INS_READ_PRESS
//#define INS_READ_LONLAT
//#define INS_READ_GPS
#define INS_READ_QUAT

// Command registers address.
#define INS_SAVE 0x00
#define INS_CALSW 0x01
#define INS_RSW 0x02
#define INS_RRATE 0x03
#define INS_BAUD 0x04
#define INS_AXOFFSET 0x05
#define INS_AYOFFSET 0x06
#define INS_AZOFFSET 0x07
#define INS_GXOFFSET 0x08
#define INS_GYOFFSET 0x09
#define INS_GZOFFSET 0x0a
#define INS_HXOFFSET 0x0b
#define INS_HYOFFSET 0x0c
#define INS_HZOFFSET 0x0d
#define INS_D0MODE 0x0e
#define INS_D1MODE 0x0f
#define INS_D2MODE 0x10
#define INS_D3MODE 0x11
#define INS_D0PWMH 0x12
#define INS_D1PWMH 0x13
#define INS_D2PWMH 0x14
#define INS_D3PWMH 0x15
#define INS_D0PWMT 0x16
#define INS_D1PWMT 0x17
#define INS_D2PWMT 0x18
#define INS_D3PWMT 0x19
#define INS_IICADDR 0x1a
#define INS_LEDOFF 0x1b
#define INS_GPSBAUD 0x1c

// Data register address.
#define INS_YYMM 0x30
#define INS_DDHH 0x31
#define INS_MMSS 0x32
#define INS_MS 0x33
#define INS_AX 0x34
#define INS_AY 0x35
#define INS_AZ 0x36
#define INS_GX 0x37
#define INS_GY 0x38
#define INS_GZ 0x39
#define INS_HX 0x3a
#define INS_HY 0x3b
#define INS_HZ 0x3c
#define INS_ROLL 0x3d
#define INS_PITCH 0x3e
#define INS_YAW 0x3f
#define INS_TEMP 0x40
#define INS_D0STATUS 0x41
#define INS_D1STATUS 0x42
#define INS_D2STATUS 0x43
#define INS_D3STATUS 0x44
#define INS_PRESSUREL 0x45
#define INS_PRESSUREH 0x46
#define INS_HEIGHTL 0x47
#define INS_HEIGHTH 0x48
#define INS_LONL 0x49
#define INS_LONH 0x4a
#define INS_LATL 0x4b
#define INS_LATH 0x4c
#define INS_GPSHEIGHT 0x4d
#define INS_GPSYAW 0x4e
#define INS_GPSVL 0x4f
#define INS_GPSVH 0x50
#define INS_Q0 0x51
#define INS_Q1 0x52
#define INS_Q2 0x53
#define INS_Q3 0x54

// Pin modes.
#define INS_DIO_MODE_AIN 0
#define INS_DIO_MODE_DIN 1
#define INS_DIO_MODE_DOH 2
#define INS_DIO_MODE_DOL 3
#define INS_DIO_MODE_DOPWM 4
#define INS_DIO_MODE_GPS 5

// Data macros and structs.
#define INS_DATA_LEN 8 // bytes of each chunk
#define INS_SCALE_TEMP (1.0 / 100.0)
#define INS_SCALE_ACC (16.0 / 32768.0)
#define INS_SCALE_GYRO (2000.0 / 32768.0)
#define INS_SCALE_ANGLE (180.0 / 32768.0)
#define INS_SCALE_ALTITUDE (1.0 / 100.0)
#define INS_SCALE_GPSH (1.0 / 10.0)
#define INS_SCALE_GPSY (1.0 / 10.0)
#define INS_SCALE_GPSV (1.0 / 1000.0)
#define INS_SCALE_QUAT (1.0 / 32768.0)

struct Time_s
{
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
};

struct Acc_s
{
    int16_t a[3];
    int16_t t;
};

struct Gyro_s
{
    int16_t w[3];
    int16_t t;
};

struct Mag_s
{
    int16_t h[3];
    int16_t t;
};

struct Angle_s
{
    int16_t angle[3];
    int16_t t;
};

struct DStatus_s
{
    int16_t DStatus[4];
};

struct Press_s
{
    int32_t pressure;
    int32_t altitude;
};

struct LonLat_s
{
    int32_t lon;
    int32_t lat;
};

struct GPS_s
{
    int16_t GPSHeight;
    int16_t GPSYaw;
    int32_t GPSVelocity;
};

struct Quat_s
{
    int16_t q[4];
};

struct RawData_s
{
    struct Time_s time;
    struct Acc_s acc;
    struct Gyro_s gyro;
    struct Mag_s mag;
    struct Angle_s angles;
    struct DStatus_s dstatus;
    struct Press_s press;
    struct LonLat_s lonlat;
    struct GPS_s gpsv;
    struct Quat_s quat;
};

struct INS_s
{
    struct RawData_s raw_data;
    struct Time_s time;
    float temperature;
    float acceleration[3];
    float angular_velocity[3];
    uint16_t magnetic_field[3];
    float angle[3];
    int16_t dstatus[4];
    float pressure;
    float altitude;
    long longitude;
    long latitude;
    float gps_height;
    float gps_yaw;
    float gps_velocity;
    float quaternion[4];
};

static inline void read_register(uint8_t, uint8_t[], uint8_t);
static inline void read_time(struct RawData_s *);
static inline void read_acc(struct RawData_s *);
static inline void read_gyro(struct RawData_s *);
static inline void read_mag(struct RawData_s *);
static inline void read_angle(struct RawData_s *);
static inline void read_dstatus(struct RawData_s *);
static inline void read_press(struct RawData_s *);
static inline void read_lonlat(struct RawData_s *);
static inline void read_gps(struct RawData_s *);
static inline void read_quat(struct RawData_s *);
void read_data(struct INS_s *);

#endif // INS_H