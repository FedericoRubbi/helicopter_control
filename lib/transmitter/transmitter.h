#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <cstdint>

extern "C"
{
#include "ins/ins.h"
}

#define TX_IRQ_PIN 8 // not used
#define TX_CE_PIN 9
#define TX_CSN_PIN 13

// Edit.
#define TX_PACKET_SIZE INS_PACKET_SIZE
#define TX_PAYLOAD_SIZE 32
#define TX_PAYLOAD_DATA_SIZE 31
#define TX_PAYLOAD_BUFFER_SIZE ((TX_PACKET_SIZE + TX_PAYLOAD_DATA_SIZE - 1) / TX_PAYLOAD_DATA_SIZE)

#define TX_SET_EOP 0xFF
#define TX_UNSET_EOP 0x00

// Command codes from RX to TX.
#define TX_CMD_STOP 0x00
#define TX_CMD_SETPOINT 0x01
#define TX_CMD_SET_PID0 0x02
#define TX_CMD_SET_PID1 0x03
#define TX_CMD_SET_DMAT0 0x04
#define TX_CMD_SET_DMAT1 0x05
#define TX_CMD_SET_DMAT2 0x06

const uint8_t tx_pipe_addr[] = {0x01, 0x02, 0x03, 0x04, 0x05};
const uint8_t rx_pipe_addr[] = {0x06, 0x07, 0x08, 0x09, 0x0a};

// Sensor data to be sent to receiver.
struct Packet_s
{
    uint8_t sensor_data[TX_PACKET_SIZE];
};

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

void reset_bootloader();
bool setup_radio();
void send_packet(struct Packet_s *packet);
bool read_cmd(struct RxAckPayload_s *ack_payload);

#endif // TRANSMITTER_H