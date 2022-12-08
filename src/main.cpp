#include <iostream>
#include <string.h> // memcpy
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <pico/multicore.h>

#include "transmitter/transmitter.h"
extern "C"
{
#include "control/control.h"
#include "ins/ins.h"
}

// Set inertial navigation system.
struct INS_s ins;

// Initialize system control model.
struct System sys = {
    .setpoint = {0.0f, 0.0f, 2.0f, 0.0f},
    .ctrl_bound = {
        {ACT_RS_MIN_MS, ACT_RS_MAX_MS},
        {ACT_LS_MIN_MS, ACT_LS_MAX_MS},
        {ACT_M_MIN_MS, ACT_M_MAX_MS},
        {ACT_M_MIN_MS, ACT_M_MAX_MS},
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

void setup()
{
    std::cout << "[DEBUG] Running setup procedure." << std::endl;
    stdio_init_all();
    queue_init(&us_range_queue, sizeof(float), queue_size); // initialize intercore queue

    setup_ins(&ins);
    setup_control(&sys);
    setup_radio();
}

// Execute received command.
inline void run_cmd(struct RxAckPayload_s *cmd)
{
    std::cout << "[DEBUG] Executing command: " << cmd->cmd_code << std::endl;
    switch (cmd->cmd_code)
    {
    case TX_CMD_STOP:
        stop(&sys);
        break;
    case TX_CMD_SETPOINT:
        memcpy(sys.setpoint, cmd->data, sizeof(sys.setpoint));
        break;
    case TX_CMD_SET_PID0:
        memcpy(&sys.controllers[0].k_p, cmd->data, TX_PAYLOAD_DATA_SIZE);
        break;
    case TX_CMD_SET_PID1:
        memcpy(&sys.controllers[0].k_p + TX_PAYLOAD_DATA_SIZE, cmd->data,
               sizeof(sys.controllers) - TX_PAYLOAD_DATA_SIZE); // copy only remaining data
        break;
    case TX_CMD_SET_DMAT0:
        memcpy(&sys.ctrl_dependence, cmd->data, TX_PAYLOAD_DATA_SIZE);
        break;
    case TX_CMD_SET_DMAT1:
        memcpy(&sys.ctrl_dependence + TX_PAYLOAD_DATA_SIZE, cmd->data, TX_PAYLOAD_DATA_SIZE);
        break;
    case TX_CMD_SET_DMAT2:
        memcpy(&sys.ctrl_dependence + TX_PAYLOAD_DATA_SIZE * 2, cmd->data,
               sizeof(sys.ctrl_dependence) - TX_PAYLOAD_DATA_SIZE * 2.0f); // copy only remaining data
        break;
    default:
        break;
    }
}

inline void check_cmd()
{
    static struct RxAckPayload_s cmd;
    if (read_cmd(&cmd))
        run_cmd(&cmd);
}

void core1_read_range()
{
    static float us_range;
    while (true)
    {
        read_range(&us_range); // read blocking
        // Attempt to add new reading. Note that queue should never be full.
        queue_try_add(&us_range_queue, &us_range);
    }
}

void test_INS()
{
    while (true)
    {
        std::cout << "INERTIAL NAVIGATION SYSTEM DATA" << std::endl;
        uint64_t start_timer = to_us_since_boot(get_absolute_time()); // start timer
        read_imu_data(&ins.imu);                                      // update sensor data
        uint64_t end_timer = to_us_since_boot(get_absolute_time());   // end timer
        std::cout << "Formatting data state: " << std::endl;
        format_imu_data(&ins.imu);                                    // format raw data
        std::cout << "Updating state: " << std::endl;
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
        std::cout << "___________________________________________________________\n\n"
                  << std::endl;
        sleep_ms(500);
    }
}

int main()
{
    setup();

    test_INS();

    multicore_launch_core1(core1_read_range); // launch range read on core 1

    while (true)
    {
        std::cout << "Attempting to read us sensor range." << std::endl;
        queue_try_remove(&us_range_queue, &ins.us_range); // check for a new range measurement
        std::cout << "Range read: " << ins.us_range << std::endl;

        std::cout << "[DEBUG] Reading IMU data." << std::endl;
        read_imu_data(&ins.imu);   // read blocking sensor data
        format_imu_data(&ins.imu); // format raw data

        std::cout << "[DEBUG] Updating system state." << std::endl;
        update_state(&ins);
        std::cout << "[DEBUG] New state vector: " << sys.state[0] << "\t" << sys.state[1] << "\t"
                  << sys.state[2] << "\t" << sys.state[3] << std::endl;

        std::cout << "[DEBUG] Updating control." << std::endl;
        update_control(&sys, &ins);
        std::cout << "[DEBUG] New control vector: " << sys.control[0] << "\t" << sys.control[1] << "\t"
                  << sys.control[2] << "\t" << sys.control[3] << std::endl;

        std::cout << "[DEBUG] sending packet." << std::endl;
        send_packet((struct Packet_s *)&ins.imu.raw_data); // add control to packet
        check_cmd();
        sleep_ms(1000);
    }
}