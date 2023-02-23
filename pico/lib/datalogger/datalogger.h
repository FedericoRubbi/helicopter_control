#ifndef DATALOGGER
#define DATALOGGER

#include <stdint.h>

#define LOG_UART uart0
#define LOG_TX_PIN 0
#define LOG_RX_PIN 1
#define LOG_DEFAULT_BAUDRATE 9600
#define LOG_BAUDRATE LOG_DEFAULT_BAUDRATE

void setup_datalogger();
void log_data(uint8_t *str, uint8_t len);

#endif // DATALOGGER