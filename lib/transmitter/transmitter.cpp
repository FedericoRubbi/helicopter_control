#include "transmitter.h"
#include "pico/stdlib.h"  // printf(), sleep_ms(), getchar_timeout_us(), to_us_since_boot(), get_absolute_time()
#include "pico/bootrom.h" // reset_usb_boot()
#include <tusb.h>         // tud_cdc_connected()
#include <RF24.h>         // RF24 radio object

// instantiate an object for the nRF24L01 transceiver
RF24 radio(CE_PIN, CSN_PIN); // move to main.cpp

struct Payload_s
{
    uint8_t dummy; // to be changed
} payload;

void reset_bootloader()
{
    radio.powerDown();
    reset_usb_boot(0, 0);
}

bool setup_radio()
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

    radio.setPALevel(RF24_PA_LOW); // RF24_PA_MAX is default.
    radio.setPayloadSize(sizeof(payload));

    radio.openWritingPipe(tx_pipe_addr);
    radio.openReadingPipe(1, rx_pipe_addr);

    radio.stopListening();

    radio.printDetails();
    radio.printPrettyDetails();

    return true;
}

void send_data()
{
    bool report = radio.write(&payload, sizeof(payload)); // transmit & save the report

    if (!report)
        printf("Transmission failed or timed out\n");
}

void wait_radio()
{
    while (!setup_radio()); // move to main.cpp
}