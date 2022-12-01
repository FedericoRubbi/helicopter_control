#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <iostream>
#include <math.h>
#include <transmitter/transmitter.h>

extern "C" {
#include "control/actuator.h" // to be moved to control.h
#include "control/control.h"
#include "ins/ins.h"
#include "ins/us_sensor.h" // to be moved to ins.h
}

// Initialize inertial navigation system.
struct INS_s ins;

// Initialize right and left servos.
struct Actuator_s servo[2];
struct Actuator_s motor[2];

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
        std::cout << "US sensor range:" << read_range() << std::endl;
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

void test_servos()
{
    init_actuator(&servo[0], ACT_RS_PIN);
    init_actuator(&servo[1], ACT_LS_PIN);

    for (float angle = 0;; angle += 0.005) {
        float s0 = sin(angle) * 0.3f + 1.1f;
        float s1 = cos(angle) * 0.3f + 1.45f;
        servo[0].duty = s0;
        servo[1].duty = s1;
        update_servos(&servo[0], &servo[1]);
        std::cout << "\tduty s0 (ms): " << s0 << " " << servo[0].duty << "\tduty s1 (ms): " << s1
                  << " " << servo[1].duty << std::endl;
    }
}

void test_motor_drivers()
{
    init_actuator(&motor[0], ACT_LM_PIN);
    init_actuator(&motor[1], ACT_UM_PIN);
    motor[0].duty = 0.0f;
    motor[1].duty = 0.0f;
    update_motors(&motor[0], &motor[1]);
    sleep_ms(100);
    motor[0].duty = 1.5f;
    motor[1].duty = 1.5f;
    update_motors(&motor[0], &motor[1]);

    for (;;) {
        std::cout << "Enter lower motor duty in ms: ";
        std::cin >> motor[0].duty;
        std::cout << "Enter upper motor duty in ms: ";
        std::cin >> motor[1].duty;
        update_motors(&motor[0], &motor[1]);
        std::cout << "M0: " << motor[0].duty << " "
                  << "\tM1: " << motor[1].duty << std::endl;
    }
}

int main()
{
    stdio_init_all();
    setup_i2c();
    // test_INS();
    // test_servos();
    test_motor_drivers();
}
