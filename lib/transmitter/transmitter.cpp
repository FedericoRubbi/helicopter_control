#include <iostream>

#include "transmitter.h"
#include "pico/stdlib.h"
#include "pico/bootrom.h" // reset_usb_boot()
#include <tusb.h>         // tud_cdc_connected()
#include <RF24.h>

// Instantiate an object for the nRF24L01 transceiver.
RF24 radio(TX_CE_PIN, TX_CSN_PIN); // move to main.cpp

struct TxPayload_s payload_buf[TX_PAYLOAD_BUFFER_SIZE]; // payload buffer to store a packet

static void split_packet(struct Packet_s *packet);
static void cmd_feedback(struct RxAckPayload_s *ack_payload);

void reset_bootloader()
{
    radio.powerDown();
    reset_usb_boot(0, 0);
}

bool setup_radio()
{
    std::cout << "Setting up radio." << std::endl;
    // wait here until the CDC ACM (serial port emulation) is connected
    while (!tud_cdc_connected())
        sleep_ms(10);

    // initialize the transceiver on the SPI bus
    while (!radio.begin())
        ;
    radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default
    radio.enableDynamicPayloads(); // ACK payloads are dynamically sized
    radio.enableAckPayload();
    radio.openWritingPipe(tx_pipe_addr);
    radio.openReadingPipe(1, rx_pipe_addr);
    radio.stopListening();
    radio.printDetails();
    radio.printPrettyDetails();
    return true;
}

// Divide logic packet into payloads.
void split_packet(struct Packet_s *packet)
{
    for (uint8_t i = 0; i < TX_PAYLOAD_BUFFER_SIZE; ++i)
    {
        memcpy(payload_buf[i].data, packet->sensor_data + i * TX_PAYLOAD_DATA_SIZE, TX_PAYLOAD_DATA_SIZE);
        payload_buf[i].eop = TX_UNSET_EOP;
    }
    payload_buf[TX_PAYLOAD_BUFFER_SIZE - 1].eop = TX_SET_EOP; // set end-of-packet on last payload
}

// Send logic packet to receiver.
void send_packet(struct Packet_s *packet)
{
    split_packet(packet); // split packed into payloads
    for (uint8_t i = 0; i < TX_PAYLOAD_BUFFER_SIZE; ++i)
        if (radio.write(&payload_buf[i], TX_PAYLOAD_SIZE)) // best effort transmission of payloads
            std::cout << "Message sent to receiver" << std::endl;
        else
            std::cout << "Error sending message" << std::endl;
}

// Read command acknowledge packet from receiver.
bool read_cmd(struct RxAckPayload_s *ack_payload)
{
    if (radio.available())
    {
        radio.read(ack_payload, sizeof(*ack_payload));
        std::cout << "command received: " << ack_payload->cmd_code << std::endl;
        return true;
    }
    return false;
}

// Mirror received payload to RX to ensure data correctness.
void cmd_feedback(struct RxAckPayload_s *ack_payload)
{
    static TxPayload_s mirror_payload;
    uint8_t cmd = ack_payload->cmd_code;

    if (cmd >= TX_CMD_SET_PID0 && cmd <= TX_CMD_SET_DMAT2)
    {
        memcpy(mirror_payload.data, ack_payload->data, sizeof(mirror_payload.data));
        mirror_payload.eop = TX_SET_EOP;
        radio.write(ack_payload->data, sizeof(*ack_payload));
    }
}
