#ifndef RECEIVER_H
#define RECEIVER_H

#include <nRF24L01.h>
#include <printf.h>
#include <RF24.h>
#include <RF24_config.h>

#define CE_PIN 10
#define CSN_PIN 9
#define PIPE 1

#define PACKET_SIZE 90
#define PAYLOAD_SIZE 32
#define PAYLOAD_BUFFER_SIZE ((PACKET_SIZE + PAYLOAD_DATA_SIZE - 1) / PAYLOAD_DATA_SIZE)
#define PAYLOAD_DATA_SIZE 31

#define STOP 0x00
#define SETPOINT 0x01
#define SET_PID0 0x03
#define SET_PID1 0x04
#define SET_DMAT0 0x05
#define SET_DMAT1 0x06
#define SET_DMAT2 0x07

#define SET_EOP 0xFF
#define UNSET_EOP 0x00

struct Packet_s
{
    uint8_t sensor_data[PACKET_SIZE];
};
struct TxPayload_s
{
    uint8_t data[PAYLOAD_DATA_SIZE];
    uint8_t eop; // end of packet flag
} __attribute__((packed));

struct RxAckPayload_s
{
    uint8_t cmd_code; // command code
    uint8_t data[PAYLOAD_DATA_SIZE];
} __attribute__((packed));

const uint8_t tx_pipe_addr[] = {0x01, 0x02, 0x03, 0x04, 0x05};
const uint8_t rx_pipe_addr[] = {0x06, 0x07, 0x08, 0x09, 0x0a};

void readPacket();
void sendAck(uint8_t *data);
void serialEvent();

#endif // RECEIVER_H
