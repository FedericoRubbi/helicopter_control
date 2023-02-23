#include "datalogger.h"
#include "pico/stdlib.h"

void setup_datalogger()
{
    uart_init(LOG_UART, LOG_BAUDRATE);
    gpio_set_function(LOG_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LOG_RX_PIN, GPIO_FUNC_UART);
}

// Write data on microSD with serial protocol
void log_data(uint8_t *str, uint8_t len)
{
    if (uart_is_writable(LOG_UART)) // false if buffer is full
        uart_write_blocking(LOG_UART, str, len);
}