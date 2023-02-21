#ifndef SERVER_H
#define SERVER_H

#include <cstdint>

// Serial communication macros definitions and constants.
#define UART uart1 // serial port 1
#define BAUDRATE 115200
#define TX_PIN 0
#define RX_PIN 1
#define PACKET_SIZE 64 // arbitrary length
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

// Preamble sent to initiate serial communication.
const uint8_t preamble[] = {0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF};

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

typedef struct Cmd_s Cmd_t;

#endif // SERVER_H