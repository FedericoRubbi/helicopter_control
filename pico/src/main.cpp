#include <iostream>
#include <string.h> // memcpy
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <pico/multicore.h>
#include <math.h>

#include "transmitter/transmitter.h"
#include "test_module/test_module.h"

extern "C"
{
#include "datalogger/datalogger.h"
#include "control/control.h"
#include "ins/ins.h"
}

// Set inertial navigation system.
struct INS_s ins;

// Initialize system control model.
struct System_s sys = {
    .setpoint = {0.0f, 0.0f, 2.0f, 0.0f},
    .ctrl_bound = {
        {ACT_RS_MIN_MS, ACT_RS_MAX_MS},
        {ACT_LS_MIN_MS, ACT_LS_MAX_MS},
        {ACT_M_MIN, ACT_M_MAX},
        {ACT_M_MIN, ACT_M_MAX},
    },
    .controllers = {
        {0.3f, 0.15f, 0.05f},
        {0.3f, 0.15f, 0.05f},
        {0.3f, 0.15f, 0.05f},
        {0.3f, 0.15f, 0.05f},
    },
    // Dependence matrix expresses physical control vector as a linear combination of state-related PID outputs.
    // Columns are weights of state-related PID outputs: x and y axes acceleration, height and yaw.
    // Rows are weights of physical control: right and left servo, lower and upper motor control.
    .ctrl_dependence = {
        {0.0f, 0.0f, -1.0f, 1.0f},  // right servo
        {0.0f, 0.0f, -1.0f, -1.0f}, // left servo
        {1.0f, 1.0f, 0.0f, 0.0f},   // lower motor
        {1.0f, -1.0f, 0.0f, 0.0f},  // upper motor
    }};

// Intercore queue data.
queue_t us_range_queue;       // intercore queue for ultrasonic sensor readings
const uint8_t queue_size = 8; // maximum number of entries in the queue

// Setup system and all required components.
void setup()
{
    std::cout << "[DEBUG] Running setup procedure." << std::endl;
    stdio_init_all();
    queue_init(&us_range_queue, sizeof(float), queue_size); // initialize intercore queue
    setup_ins(&ins);
    setup_control(&sys);
    setup_datalogger();
    setup_transmitter();
}

// Reading procedure running on the second core.
void core1_read_range()
{
    for (float us_range;; read_range(&us_range))   // read blocking
        queue_try_add(&us_range_queue, &us_range); // attempt to add new reading. Queue should never be full!
}

int main()
{
    setup();
    sleep_ms(10000);
    test_servos(&sys);
    // test_flight(&sys);
    std::cout << "End of test" << std::endl;
    while (1)
        ; // hang execution

    multicore_launch_core1(core1_read_range); // take off and hover at minimum sufficient power
    // takeoff(&sys);                            // take off and hover at minimum sufficient power

    while (true)
    {
        std::cout << "Attempting to read us sensor range." << std::endl;
        queue_try_remove(&us_range_queue, &ins.us_range); // check for a new range measurement
        std::cout << "Range read: " << ins.us_range << std::endl;

        std::cout << "[DEBUG] Reading IMU data." << std::endl;
        read_imu_data(&ins.imu);                                 // read blocking sensor data
        format_imu_data(&ins.imu);                               // format raw data
        log_data((uint8_t *)&ins.imu.raw_data, INS_PACKET_SIZE); // save sensor data on microSD card

        std::cout << "[DEBUG] Updating system state." << std::endl;
        update_state(&ins);
        std::cout << "[DEBUG] New state vector: " << sys.state[0] << "\t" << sys.state[1] << "\t"
                  << sys.state[2] << "\t" << sys.state[3] << std::endl;

        // std::cout << "[DEBUG] Updating control." << std::endl;
        // update_control(&sys, &ins);
        // std::cout << "[DEBUG] New control vector: " << sys.control[0] << "\t" << sys.control[1] << "\t"
        //           << sys.control[2] << "\t" << sys.control[3] << std::endl;

        std::cout << "[DEBUG] sending packet." << std::endl;
        send_packet((struct Packet_s *)&ins.imu.raw_data);
        check_cmd(&sys);
        sleep_ms(500);
    }
}

int main_for_presentation()
{
    setup();
    multicore_launch_core1(core1_read_range); // start ultrasonic sensor readings on the second core
    takeoff(&sys);                            // take off and hover at minimum sufficient power

    while (true) // main loop
    {
        queue_try_remove(&us_range_queue, &ins.us_range); // check for a new range measurement

        read_imu_data(&ins.imu);                                 // read sensor data (blocking)
        format_imu_data(&ins.imu);                               // format raw data
        log_data((uint8_t *)&ins.imu.raw_data, INS_PACKET_SIZE); // save sensor data on microSD card
        update_state(&ins);                                      // compute current state given new sensor data

        // update_control(&sys, &ins); // control algorithm not tested

        send_packet((struct Packet_s *)&ins.imu.raw_data);
        check_cmd(&sys); // check and execute commands
    }
}