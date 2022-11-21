#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <iostream>
#include <transmitter/transmitter.h>

extern "C" {
#include "ins/ins.h"
#include "control/control.h"
}

// Initialize Inertial navigation system.
struct INS_s ins;

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
        //printf("US sensor range: %f\n", read_range());
        std::cout << "INERTIAL NAVIGATION SYSTEM DATA" << std::endl;
        uint64_t start_timer = to_us_since_boot(get_absolute_time()); // start timer
        read_imu_data(&ins.imu); // update sensor data
        uint64_t end_timer = to_us_since_boot(get_absolute_time()); // end timer
        format_imu_data(&ins.imu); // format raw data
        update_state(&ins);
        std::cout << "Data read and format time: " << end_timer - start_timer << " us" << std::endl;
        std::cout << "\t time: " << unsigned(ins.imu.time.year) << "/"
                  << unsigned(ins.imu.time.month) << "/" << unsigned(ins.imu.time.day) << ":"
                  << unsigned(ins.imu.time.second) << "." << ins.imu.time.millisecond << std::endl;

        std::cout << "\t acceleration: " << ins.imu.acceleration[0] << "\t"
                  << ins.imu.acceleration[1] << "\t" << ins.imu.acceleration[2] << std::endl;
        std::cout << "\t angle: " << ins.imu.angle[0] << "\t" << ins.imu.angle[1] << "\t"
                  << ins.imu.angle[2] << std::endl;

        std::cout << "\t angular velocity: " << ins.imu.angular_velocity[0] << "\t"
                  << ins.imu.angular_velocity[1] << "\t" << ins.imu.angular_velocity[2]
                  << std::endl;

        // std::cout << "\t magnetic field: " << ins.imu.magnetic_field[0] << "\t"
        //           << ins.imu.magnetic_field[1] << "\t" << ins.imu.magnetic_field[2] << std::endl;
        // std::cout << "\t temperature: " << ins.imu.temperature << std::endl;
        // std::cout << "\t pin input: " << ins.imu.dstatus[0] << "\t" << ins.imu.dstatus[1] << "\t"
        //           << ins.imu.dstatus[2] << "\t" << ins.imu.dstatus[3] << std::endl;
        // std::cout << "\t pressure: " << ins.imu.pressure << std::endl;
        // std::cout << "\t altitude: " << ins.imu.altitude << std::endl;
        // std::cout << "\t longitude: " << ins.imu.longitude << std::endl;
        // std::cout << "\t latitude: " << ins.imu.latitude << std::endl;
        // std::cout << "\t gps velocity: " << ins.imu.gps_velocity << std::endl;
        // std::cout << "\t gps yaw: " << ins.imu.gps_yaw << std::endl;
        // std::cout << "\t gps height: " << ins.imu.gps_height << std::endl;

        std::cout << "\t quaternions: " << ins.imu.quaternion[0] << "\t" << ins.imu.quaternion[1]
                  << "\t" << ins.imu.quaternion[2] << "\t" << ins.imu.quaternion[3] << std::endl;
        std::cout << "___________________________________________________________\n\n" << std::endl;
        sleep_ms(500);
    }
}

int main()
{
    stdio_init_all();
    setup_i2c();
    test_INS();
    //test_US();
}
