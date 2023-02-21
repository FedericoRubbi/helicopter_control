#include <cmath>
#include <iostream>
#include <pico/stdlib.h>

#include "test_module.h"
#include "transmitter/transmitter.h"

extern "C"
{
#include "ins/ins.h"
#include "control/actuator.h"
#include "control/control.h"
}

// Test swash plate servos given a system after setup procedure.
void test_servos(struct System_s *sys)
{
    std::cout << "[TESTING] Testing swash plate servos." << std::endl;
    for (float angle = 0.f;; angle += 0.05f)
    {
        sys->servo[0].duty_period = sin(angle) * ACT_S_RADIUS + ACT_RS_MIDPOINT;
        sys->servo[1].duty_period = cos(angle) * ACT_S_RADIUS + ACT_LS_MIDPOINT;
        update_servos(&sys->servo[0], &sys->servo[1]); // update right and left servos
        sleep_ms(10);
    }
}

// Test motors given a system after setup procedure.
void test_motors(struct System_s *sys)
{
    std::cout << "[TESTING] Testing motors." << std::endl;
    while (true)
    {
        sys->motor[0].duty = ACT_M_TAKEOFF * 0.8f; // set slightly lower speed
        sys->motor[1].duty = ACT_M_TAKEOFF * 0.8f;
        update_motors(&sys->motor[0], &sys->motor[1]); // update lower and upper motors
        sleep_ms(1000);
        sys->motor[0].duty = ACT_M_MIN;
        sys->motor[1].duty = ACT_M_MIN;
        update_motors(&sys->motor[0], &sys->motor[1]); // update lower and upper motors
        sleep_ms(4000);
    }
}

// Test inertial measurement system after setup procedure.
void test_INS(struct INS_s *ins)
{
    while (true)
    {
        std::cout << "[TESTING] Testing inertial measurement system." << std::endl;
        std::cout << "[TESTING] Reading and formatting data state: " << std::endl;
        uint64_t start_timer = to_us_since_boot(get_absolute_time()); // start timer
        read_imu_data(&ins->imu);                                     // update sensor data
        format_imu_data(&ins->imu);                                   // format raw data
        uint64_t end_timer = to_us_since_boot(get_absolute_time());   // end timer
        std::cout << "[TESTING] Data read and format time: " << end_timer - start_timer << " us" << std::endl;

        std::cout << "[TESTING] Updating state: " << std::endl;
        update_state(ins);

#ifdef IMU_READ_TIME
        std::cout << "\t time: " << unsigned(ins->imu.time.year) << "/"
                  << unsigned(ins->imu.time.month) << "/" << unsigned(ins->imu.time.day) << ":"
                  << unsigned(ins->imu.time.second) << "." << ins->imu.time.millisecond << std::endl;
#endif
#ifdef IMU_READ_ACC
        std::cout << "\t acceleration: " << ins->imu.acceleration[0] << "\t"
                  << ins->imu.acceleration[1] << "\t" << ins->imu.acceleration[2] << std::endl;
#endif
#ifdef IMU_READ_ANGLE
        std::cout << "\t angle: " << ins->imu.angle[0] << "\t" << ins->imu.angle[1] << "\t"
                  << ins->imu.angle[2] << std::endl;
#endif
#ifdef IMU_READ_GYRO
        std::cout << "\t angular velocity: " << ins->imu.angular_velocity[0] << "\t"
                  << ins->imu.angular_velocity[1] << "\t" << ins->imu.angular_velocity[2]
                  << std::endl;
#endif
#ifdef IMU_READ_MAG
        std::cout << "\t magnetic field: " << ins->imu.magnetic_field[0] << "\t"
                  << ins->imu.magnetic_field[1] << "\t" << ins->imu.magnetic_field[2] << std::endl;
#endif
#ifdef IMU_READ_TEMP
        std::cout << "\t temperature: " << ins->imu.temperature << std::endl;
#endif
#ifdef IMU_READ_DSTATUS
        std::cout << "\t pin input: " << ins->imu.dstatus[0] << "\t" << ins->imu.dstatus[1] << "\t"
                  << ins->imu.dstatus[2] << "\t" << ins->imu.dstatus[3] << std::endl;
#endif
#ifdef IMU_READ_PRESS
        std::cout << "\t pressure: " << ins->imu.pressure << std::endl;
#endif
#ifdef IMU_READ_ALTITUDE
        std::cout << "\t altitude: " << ins->imu.altitude << std::endl;
#endif
#ifdef IMU_READ_LONLAT
        std::cout << "\t longitude: " << ins->imu.longitude << std::endl;
        std::cout << "\t latitude: " << ins->imu.latitude << std::endl;
#endif
#ifdef IMU_READ_GPS
        std::cout << "\t gps velocity: " << ins->imu.gps_velocity << std::endl;
        std::cout << "\t gps yaw: " << ins->imu.gps_yaw << std::endl;
        std::cout << "\t gps height: " << ins->imu.gps_height << std::endl;
#endif
#ifdef IMU_READ_QUAT
        std::cout << "\t quaternions: " << ins->imu.quaternion[0] << "\t" << ins->imu.quaternion[1]
                  << "\t" << ins->imu.quaternion[2] << "\t" << ins->imu.quaternion[3] << std::endl;
#endif
        std::cout << "________________________________________________________________________________\n\n"
                  << std::endl;
        sleep_ms(500);
    }
}

// Test ultrasonic sensor after setup procedure.
void test_us_sensor()
{
    std::cout << "[TESTING] Testing ultrasonic sensor." << std::endl;
    for (float range = 0.0f;; read_range(&range))
        std::cout << "\t range: " << range << std::endl;
}
