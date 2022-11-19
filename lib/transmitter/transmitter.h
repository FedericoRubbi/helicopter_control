#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <cstdint>

#define IRQ_PIN 8 // not used
#define CE_PIN 9
#define CSN_PIN 13

// Edit.
#define PACKET_SIZE 90
#define PAYLOAD_SIZE 32
#define PAYLOAD_BUFFER_SIZE ((PACKET_SIZE + PAYLOAD_DATA_SIZE -1 ) /PAYLOAD_DATA_SIZE)
#define PAYLOAD_DATA_SIZE 31

#define SET_EOP 0xFF

#define UNSET_EOP 0x00

// Command codes from RX to TX.
#define STOP 0x00
#define SETPOINT 0x01
#define SAVE 0x02
#define SET_PID0 0x03
#define SET_PID1 0x04
#define SET_DMAT0 0x05
#define SET_DMAT1 0x06
#define SET_DMAT2 0x07

const uint8_t tx_pipe_addr[] = {0x01, 0x02, 0x03, 0x04, 0x05};
const uint8_t rx_pipe_addr[] = {0x06, 0x07, 0x08, 0x09, 0x0a};

// Sensor data to be sent to receiver.
struct Packet_s
{
    uint8_t sensor_data[PACKET_SIZE];
};
struct TxPayload_s
{
    uint8_t data[PAYLOAD_DATA_SIZE];
    uint8_t eop; // end of packet flag
} __packed;

struct RxAckPayload_s
{
    uint8_t data[PAYLOAD_DATA_SIZE];
    uint8_t cmd_code; // command code
} __packed;

void resetBootloader();
bool setupRadio();
void sendPacket();
void waitRadio();
void readAck();
void splitPacket(struct Packet_s *packet);

#endif // TRANSMITTER_H