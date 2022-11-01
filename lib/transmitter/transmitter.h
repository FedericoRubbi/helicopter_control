#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <cstdint>

#define IRQ_PIN 8 // not used
#define CE_PIN 9
#define CSN_PIN 13

const uint8_t tx_pipe_addr[] = {0x01, 0x02, 0x03, 0x04, 0x05};
const uint8_t rx_pipe_addr[] = {0x06, 0x07, 0x08, 0x09, 0x0a};

void reset_bootloader();
bool setup_radio();
void send_data();
void wait_radio();

#endif // TRANSMITTER_H