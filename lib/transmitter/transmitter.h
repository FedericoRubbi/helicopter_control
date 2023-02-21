#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <cstdint>

extern "C"
{
#include "ins/ins.h"
}

// Macros to select transmitter mode. Only one mode can be active.
#define TX_SERIAL // use uart protocol
//#define TX_RF24   // use spi protocol and RF24 lib

#define TX_PACKET_SIZE INS_PACKET_SIZE

// RF24 macros definitions and constants.
#ifdef TX_RF24
#define TX_IRQ_PIN 8 // not used
#define TX_CE_PIN 9
#define TX_CSN_PIN 13
#define TX_PAYLOAD_MAX_SIZE 32 // max length by implementation
#define TX_PAYLOAD_SIZE TX_PAYLOAD_MAX_SIZE
#define TX_PAYLOAD_DATA_SIZE (TX_PAYLOAD_SIZE - 1)
#define TX_PAYLOAD_BUFFER_SIZE ((TX_PACKET_SIZE + TX_PAYLOAD_DATA_SIZE - 1) / TX_PAYLOAD_DATA_SIZE)
#define TX_SET_EOP 0xFF
#define TX_UNSET_EOP 0x00
// Command codes from receiver to transmitter.
#define TX_CMD_STOP 0x00
#define TX_CMD_SETPOINT 0x01
#define TX_CMD_SET_PID0 0x02
#define TX_CMD_SET_PID1 0x03
#define TX_CMD_SET_DMAT0 0x04
#define TX_CMD_SET_DMAT1 0x05
#define TX_CMD_SET_DMAT2 0x06

const uint8_t tx_pipe_addr[] = {0x01, 0x02, 0x03, 0x04, 0x05};
const uint8_t rx_pipe_addr[] = {0x06, 0x07, 0x08, 0x09, 0x0a};
#endif

// Serial communication macros definitions and constants.
#ifdef TX_SERIAL
#define TX_UART uart1 // serial port 1
#define TX_BAUDRATE 115200
#define TX_TX_PIN 8
#define TX_RX_PIN 9
#define TX_CMD_DATA_SIZE 32 // arbitrary length
// Command codes from receiver to transmitter.
#define TX_CMD_STOP 0x00
#define TX_CMD_SETPOINT 0x01
#define TX_CMD_SET_PID0 0x02
#define TX_CMD_SET_PID1 0x03
#define TX_CMD_SET_DMAT0 0x04
#define TX_CMD_SET_DMAT1 0x05
#define TX_CMD_SET_DMAT2 0x06
#define TX_CMD_TAKEOFF 0x07
#define TX_CMD_LAND 0x08
#endif

// Sensor data to be sent to receiver.
struct Packet_s
{
    uint8_t sensor_data[TX_PACKET_SIZE];
};

// RF24 structures.
#ifdef TX_RF24
struct TxPayload_s
{
    uint8_t data[TX_PAYLOAD_DATA_SIZE];
    uint8_t eop; // end of packet flag
} __attribute__((packed));
struct RxAckPayload_s
{
    uint8_t cmd_code; // command code
    uint8_t data[TX_PAYLOAD_DATA_SIZE];
} __attribute__((packed));
typedef struct RxAckPayload_s Cmd_t;
#endif

// Serial structures.
#ifdef TX_SERIAL
struct Cmd_s
{
    uint8_t cmd_code; // command code
    uint8_t data[TX_CMD_DATA_SIZE];
} __attribute__((packed));
typedef struct Cmd_s Cmd_t;
#endif

#ifdef RF24
void reset_bootloader();
bool setup_transmitter();
#endif
bool setup_transmitter();
void send_packet(struct Packet_s *packet);
bool read_cmd(Cmd_t *cmd);
void run_cmd(struct System_s *sys, Cmd_t *cmd);
void check_cmd(struct System_s *sys);

#endif // TRANSMITTER_H