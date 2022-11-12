#include "hardware/i2c.h"

#include "ins.h"

// Write register address to read and read resonse bytes from INS address
static inline void read_register(uint8_t addr, uint8_t buffer[], uint8_t buffer_size)
{
    // Write address of register to read.
    i2c_write_blocking(i2c1, INS_DEFAULT_ADDR, &addr, 1,
        true); // nostop set to keep control of the bus
    // Fill buffer with data.
    i2c_read_blocking(i2c1, INS_DEFAULT_ADDR, buffer, buffer_size,
        false); // nostop unset to release bus
}

// COpy scaled data in standard units to INS struct.
void format_data(struct INS_s* ins)
{
#ifdef INS_READ_TIME
    ins->time = ins->raw_data.time;
#endif
#ifdef INS_READ_ACC
    for (uint8_t i = 0; i < 3; ++i)
        ins->acceleration[i] = (double)(ins->raw_data.acc[i] * INS_SCALE_ACC);
#endif
#ifdef INS_READ_GYRO
    for (uint8_t i = 0; i < 3; ++i)
        ins->angular_velocity[i] = (float)ins->raw_data.gyro[i] * INS_SCALE_GYRO;
#endif
#ifdef INS_READ_MAG
    for (uint8_t i = 0; i < 3; ++i)
        ins->magnetic_field[i] = ins->raw_data.mag[i];
#endif
#ifdef INS_READ_ANGLE
    for (uint8_t i = 0; i < 3; ++i)
        ins->angle[i] = (float)ins->raw_data.angle[i] * INS_SCALE_ANGLE;
#endif
#ifdef INS_READ_TEMP
    ins->temperature = (float)ins->raw_data.temperature * INS_SCALE_TEMP;
#endif
#ifdef INS_READ_DSTATUS
    for (uint8_t i = 0; i < 3; ++i)
        ins->dstatus[i] = ins->raw_data.dstatus[i]; // * INS_SCALE_DSTATUS for analog input;
#endif
#ifdef INS_READ_PRESS
    ins->pressure = ins->raw_data.pressure;
#endif
#ifdef INS_READ_ALTITUDE
    ins->altitude = (float)ins->raw_data.altitude * INS_SCALE_ALTITUDE;
#endif
#ifdef INS_READ_LATLON
    ins->latitude = ins->raw_data.latitude;
    ins->longitude = ins->raw_data.longitude;
#endif
#ifdef INS_READ_GPS
    ins->gps_velocity = (float)ins->raw_data.gps_velocity * INS_SCALE_GPSV;
    ins->gps_yaw = (float)ins->raw_data.gps_yaw * INS_SCALE_GPSY;
    ins->gps_height = (float)ins->raw_data.gps_height * INS_SCALE_GPSH;
#endif
#ifdef INS_READ_QUAT
    for (uint8_t i = 0; i < 4; ++i)
        ins->quaternion[i] = (double)ins->raw_data.quat[i] * INS_SCALE_QUAT;
#endif
}

// Read all data and write it to INS_s struct.
void read_data(struct INS_s* ins)
{
    read_register(INS_YYMM, (uint8_t*)&ins->raw_data, sizeof(ins->raw_data));
}

// TESTED: WORKING.
// Compute quaternions product.
static inline void quat_prod(double q[], double p[], double r[])
{
    r[0] = q[0] * p[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3];
    r[1] = q[0] * p[1] + q[1] * p[0] + q[2] * p[3] - q[3] * p[2];
    r[2] = q[0] * p[2] + q[2] * p[0] + q[3] * p[1] - q[1] * p[3];
    r[3] = q[0] * p[3] + q[3] * p[0] + q[1] * p[2] - q[2] * p[1];
}

// Compute quaternions norm.
static inline double quat_norm(double q[])
{
    return q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
}

// Compute quaternion conjugate.
static inline void quat_conjugate(double q[], double p[])
{
    p[0] = q[0];
    for (uint8_t i = 1; i < 4; ++i)
        p[i] = -q[i];
}

// Compute quaternion inverse.
static inline void quat_inverse(double q[], double p[])
{
    double norm = quat_norm(q);
    // p is assigned to q conjugate divided by q norm.
    p[0] = q[0] / norm;
    for (uint8_t i = 1; i < 4; ++i)
        p[i] = -q[i] / norm;
}

// TODO: check formula correctness.
// TODO: implement acceleration software calibration.
// TODO: consider checking and correcting unit quaternion norm.
// Fast formula to subtract gravity.
// Gravity is assumed to be only on vertical z-axis, hence good angle calibration is determinant.
static inline void filter_gravity(struct INS_s* ins)
{
    double* q = ins->quaternion;
    double* a = ins->acceleration;
    // double norm_sq = quat_norm(q); // compute square of the norm
    // norm_sq *= norm_sq;
    a[1] -= 2.0 * (q[1] * q[3] - q[0] * q[2]) /* / norm_sq*/;
    a[2] -= 2.0 * (q[0] * q[1] + q[2] * q[3]) /* / norm_sq*/;
    a[3] -= - q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3] /* / norm_sq*/;
}

// Compute velocity and position with trapezoidal rule integration.
void update_state(struct INS_s* ins)
{
    filter_gravity(ins);
    // Integrate acceleration to compute position.
    for (uint8_t i = 0; i < 4; ++i) {
        ins->velocity[i] += ins->acceleration[i] * INS_SAMPLE_RATE * INS_SCALE_VELOCITY;
        ins->position[i] += ins->velocity[i] * INS_SAMPLE_RATE;
    }
}

void slow_gfilter(struct INS_s* ins)
{
    const double g[4] = { 0.0, 0.0, 0.0, 1.0 };
    double q_inverse[4];
    quat_inverse(ins->quaternion, q_inverse);
    double q_inverse_conj[4];
    quat_conjugate(q_inverse, q_inverse_conj);
    double q_res[4];
    quat_prod(q_inverse, g, q_res);
    double q_res1[4];
    quat_prod(q_res, q_inverse_conj, q_res1);

    for (uint8_t i = 0; i < 4; ++i) // subtract rotated gravity
        ins->acceleration[i] -= q_res1[i];
}
