#include "transmitter.h"
#include "pico/stdlib.h"  // printf(), sleep_ms(), getchar_timeout_us(), to_us_since_boot(), get_absolute_time()
#include "pico/bootrom.h" // reset_usb_boot()
#include <tusb.h>         // tud_cdc_connected()
#include <RF24.h>         // RF24 radio object

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN); // move to main.cpp

struct TxPayload_s payload_buf[PAYLOAD_BUFFER_SIZE]; // payload buffer to store a packet

void resetBootloader()
{
    radio.powerDown();
    reset_usb_boot(0, 0);
}

bool setupRadio()
{
    // wait here until the CDC ACM (serial port emulation) is connected
    while (!tud_cdc_connected())
    {
        sleep_ms(10);
    }

    // initialize the transceiver on the SPI bus
    if (!radio.begin())
    {
        printf("radio hardware is not responding!!\n");
        return false;
    }

    radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default

    // radio.setPayloadSize(sizeof(payload)); // static payload size

    // Enable dinamic payload and ack payload.
    radio.enableDynamicPayloads(); // ACK payloads are dynamically sized
    // Acknowledgement packets have no payloads by default. We need to enable
    // this feature for all nodes (TX & RX) to use ACK payloads.
    radio.enableAckPayload();

    radio.openWritingPipe(tx_pipe_addr);
    radio.openReadingPipe(1, rx_pipe_addr);

    radio.stopListening();

    radio.printDetails();
    radio.printPrettyDetails();

    return true;
}

void sendPacket()
{
    for(uint8_t i = 0; i < PAYLOAD_BUFFER_SIZE; ++i){
        bool report = radio.write(&payload_buf[i], PAYLOAD_SIZE);
        if (!report)
        printf("Transmission failed or timed out\n");
    }
}

void waitRadio()
{
    while (!setupRadio())
        ; // move to main.cpp
}

void readAck(struct RxAckPayload_s *ack_payload)
{
    if (radio.available())
    { // is there an ACK payload? grab the pipe number that received it
        radio.read(ack_payload, sizeof(*ack_payload));
    }

}

// Mirror received payload to RX to ensure data correctness.
void cmdFeedback(struct RxAckPayload_s *ack_payload)
{
    uint8_t cmd = ack_payload->cmd_code;
    if(cmd >= SET_PID0 && cmd <= SET_DMAT2)
        radio.write(ack_payload->data, sizeof(*ack_payload));
}

void splitPacket(struct Packet_s *packet)
{
    for(uint8_t i = 0; i < PAYLOAD_BUFFER_SIZE; ++i){
        memcpy(payload_buf[i].data, packet->sensor_data + i * PAYLOAD_DATA_SIZE, PAYLOAD_DATA_SIZE);
        payload_buf[i].eop = UNSET_EOP;
    }
    
    payload_buf[PAYLOAD_BUFFER_SIZE -1].eop = SET_EOP;
}
