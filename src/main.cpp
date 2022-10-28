#include <stdio.h>
#include "pico/stdlib.h" // printf(), sleep_ms(), getchar_timeout_us(), to_us_since_boot(), get_absolute_time()
#include <transmitter/transmitter.h>
#include "hardware/i2c.h"

extern "C"
{
#include "ins/ins.h"
}

// Initialize Inertial navigation system.
struct INS_s INS;

void setup_i2c()
{
    printf("Initializing I2C...\n");
    i2c_init(i2c1, INS_DEFAULT_BAUDRATE);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

int main()
{
    stdio_init_all();
    setup_i2c();
    while (true)
    {
        uint64_t start_timer = to_us_since_boot(get_absolute_time()); // start timer
        read_data(&INS);                                              // update sensor data
        uint64_t end_timer = to_us_since_boot(get_absolute_time());   // end timer
        printf("Data read time: %llu us\n", end_timer - start_timer);
        // printf("INS temperature: %f\n", INS.temperature);
        printf("INS temperature: %f\n", (float)INS.raw_data.angles.t / 100.0);
        printf("INS altitude: %f\n", INS.altitude);
        printf("INS time: %u/%u/%u: %u.%u\n", INS.time.year, INS.time.month, INS.time.day,
               INS.time.second, INS.time.millisecond);
        printf("INS acceleration: %f\t%f\t%f\n", INS.acceleration[0], INS.acceleration[1],
               INS.acceleration[2]);
        printf("INS raw acceleration: %f\t%f\t%f\n", INS.raw_data.acc.a[0], INS.raw_data.acc.a[1],
               INS.raw_data.acc.a[2]);
        printf("###############################################################\n\n");
        sleep_ms(500);
    }
}
