#include <iostream>
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <transmitter/transmitter.h>

extern "C" {
#include "ins/ins.h"
}

// Initialize Inertial navigation system.
struct INS_s INS;

void setup_i2c()
{
    std::cout << "Initializing I2C..." << std::endl;
    i2c_init(i2c1, INS_DEFAULT_BAUDRATE);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

void test_INS()
{
    while (true) {
        std::cout << "INERTIAL NAVIGATION SYSTEM DATA" << std::endl;
        uint64_t start_timer = to_us_since_boot(get_absolute_time()); // start timer
        read_data(&INS); // update sensor data
        uint64_t end_timer = to_us_since_boot(get_absolute_time()); // end timer
        format_data(&INS); // format raw data
        std::cout << "Data read and format time: " << end_timer - start_timer << " us" << std::endl;
        std::cout << "\t time: " << unsigned(INS.time.year) << "/" << unsigned(INS.time.month)
                  << "/" << unsigned(INS.time.day) << ":" << unsigned(INS.time.second) << "."
                  << INS.time.millisecond << std::endl;

        std::cout << "\t acceleration: " << INS.acceleration[0] << "\t" << INS.acceleration[1]
                  << "\t" << INS.acceleration[2] << std::endl;
        std::cout << "\t angular velocity: " << INS.angular_velocity[0] << "\t"
                  << INS.angular_velocity[1] << "\t" << INS.angular_velocity[2] << std::endl;
        std::cout << "\t magnetic field: " << INS.magnetic_field[0] << "\t" << INS.magnetic_field[1]
                  << "\t" << INS.magnetic_field[2] << std::endl;
        std::cout << "\t angle: " << INS.angle[0] << "\t" << INS.angle[1] << "\t" << INS.angle[2]
                  << std::endl;

        std::cout << "\t temperature: " << INS.temperature << std::endl;

        std::cout << "\t pin input: " << INS.dstatus[0] << "\t" << INS.dstatus[1] << "\t"
                  << INS.dstatus[2] << "\t" << INS.dstatus[3] << std::endl;

        std::cout << "\t pressure: " << INS.pressure << std::endl;
        std::cout << "\t altitude: " << INS.altitude << std::endl;

        std::cout << "\t longitude: " << INS.longitude << std::endl;
        std::cout << "\t latitude: " << INS.latitude << std::endl;

        std::cout << "\t gps velocity: " << INS.gps_velocity << std::endl;
        std::cout << "\t gps yaw: " << INS.gps_yaw << std::endl;
        std::cout << "\t gps height: " << INS.gps_height << std::endl;

        std::cout << "\t quaternions: " << INS.quaternion[0] << "\t" << INS.quaternion[1] << "\t"
                  << INS.quaternion[2] << "\t" << INS.quaternion[3] << std::endl;

        std::cout << "___________________________________________________________\n\n" << std::endl;
        sleep_ms(500);
    }
}

int main()
{
    stdio_init_all();
    setup_i2c();
    test_INS();
}
