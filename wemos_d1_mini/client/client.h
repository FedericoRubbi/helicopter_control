#ifndef CLIENT_H
#define CLIENT_H

#include <cstdint>

// Serial communication macros definitions and constants.
#define BAUDRATE 115200
#define TX_PIN 0
#define RX_PIN 1
#define PACKET_SIZE (sizeof(struct RawData_s)) // arbitrary length
#define PREAMBLE_SIZE (sizeof(preamble))
#define CONNECTED_FLAG 0xCC // expected receiver response after preamble
#define EOP_FLAG 0xFF       // sent after every packet for synchronization
#define CMD_DATA_SIZE 32    // arbitrary length

// Command codes from receiver to transmitter.
#define CMD_STOP 0x00
#define CMD_SETPOINT 0x01
#define CMD_SET_PID0 0x02
#define CMD_SET_PID1 0x03
#define CMD_SET_DMAT0 0x04
#define CMD_SET_DMAT1 0x05
#define CMD_SET_DMAT2 0x06
#define CMD_TAKEOFF 0x07
#define CMD_LAND 0x08

// Wi-Fi related macros.
#define SERVER_IP "192.168.50.95" // controller server ip
#ifndef STASSID
#define STASSID ""
#define STAPSK ""
#endif

// Sensor data read options. Uncomment to select.
#define READ_TIME
#define READ_ACC
#define READ_GYRO
// #define READ_MAG // not needed
#define READ_ANGLE
// #define READ_TEMP // not needed
// #define READ_DSTATUS // not needed
// #define READ_PRESS // not needed
#define READ_ALTITUDE
// #define READ_LONLAT // not needed
// #define READ_GPS // not needed
#define READ_QUAT

// Preamble sent to initiate serial communication.
const uint8_t preamble[] = {0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF};

typedef struct Cmd_s Cmd_t;

#ifdef READ_TIME
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
#endif

struct RawData_s
{
#ifdef READ_TIME
    struct Time_s time;
#endif
#ifdef READ_ACC
    int16_t acc[3];
#endif
#ifdef READ_GYRO
    int16_t gyro[3];
#endif
#ifdef READ_MAG
    int16_t mag[3];
#endif
#ifdef READ_ANGLE
    int16_t angle[3];
#endif
#ifdef READ_TEMP
    int16_t temperature;
#endif
#ifdef READ_DSTATUS
    int16_t dstatus[4];
#endif
#ifdef READ_PRESS
    uint32_t pressure;
#endif
#ifdef READ_ALTITUDE
    int32_t altitude;
#endif
#ifdef READ_LONLAT
    int32_t longitude;
    int32_t latitude;
#endif
#ifdef READ_GPS
    int16_t gps_height;
    int16_t gps_yaw;
    uint32_t gps_velocity;
#endif
#ifdef READ_QUAT
    int16_t quat[4]; // real part first, imaginary part after
#endif
} __attribute__((packed));

// Sensor data to be sent to receiver.
struct Packet_s
{
    uint8_t sensor_data[PACKET_SIZE];
};

struct Cmd_s
{
    uint8_t cmd_code; // command code
    uint8_t data[CMD_DATA_SIZE];
} __attribute__((packed));

#endif // CLIENT_H