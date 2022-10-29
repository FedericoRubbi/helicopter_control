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

/*
struct INS_s // 74 + 106 bytes
{
    struct RawData_s raw_data;
    struct Time_s time;
    float acceleration[3];
    float angular_velocity[3];
    int16_t magnetic_field[3];
    float angle[3];
    float temperature;
    int16_t dstatus[4];
    float pressure;
    float altitude;
    int32_t longitude;
    int32_t latitude;
    float gps_height;
    float gps_yaw;
    float gps_velocity;
    float quaternion[4];
};
*/

int main()
{
    stdio_init_all();
    setup_i2c();
    while (true)
    {
        uint64_t start_timer = to_us_since_boot(get_absolute_time()); // start timer
        read_data(&INS);                                              // update sensor data
        uint64_t end_timer = to_us_since_boot(get_absolute_time());   // end timer
        printf("INERTIAL NAVIGATION SYSTEM DATA\n");
        printf("Data read time: %llu us\n", end_timer - start_timer);
        printf("\t time: %u/%u/%u: %u.%u\n", INS.time.year, INS.time.month, INS.time.day,
               INS.time.second, INS.time.millisecond);

        printf("\t acceleration: %f\t%f\t%f\n", INS.acceleration[0], INS.acceleration[1],
               INS.acceleration[2]);

        printf("\t angular velocity: %f\t%f\t%f\n", INS.angular_velocity[0],
               INS.angular_velocity[1], INS.angular_velocity[2]);

        printf("\t magnetic field: %f\t%f\t%f\n", INS.magnetic_field[0],
               INS.magnetic_field[1], INS.magnetic_field[2]);

        printf("\t angle: %f\t%f\t%f\n", INS.angle[0], INS.angle[1], INS.angle[2]);

        printf("\t temperature: %f\n", INS.temperature);

        printf("\t pin input: %d\t%d\t%d\t%d\n", INS.dstatus[0], INS.dstatus[1], INS.dstatus[2],
               INS.dstatus[4]);

        printf("\t pressure: %f\n", INS.pressure);

        printf("\t altitude: %f\n", INS.altitude);

        printf("\t longitude: %d\n", INS.longitude);

        printf("\t latitude: %d\n", INS.latitude);

        printf("\t gps velocity: %f\n", INS.gps_velocity);

        printf("\t gps yaw: %f\n", INS.gps_yaw);

        printf("\t gps height: %f\n", INS.gps_height);

        printf("\t quaternions: %f\t%f\t%f\t%f\n", INS.quaternion[0], INS.quaternion[1], INS.quaternion[2],
               INS.quaternion[4]);

        printf("###############################################################\n\n\n");
        sleep_ms(2000);
    }
}
