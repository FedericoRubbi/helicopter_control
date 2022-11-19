// Header to test quaternions computation. Delete when no longer needed.
#ifndef INS_H
#define INS_H

#include <cstdint>

#include "ins.h"

// TESTED: WORKING.
// Compute quaternions product.
inline void quat_prod(double q[], double p[], double r[])
{
    r[0] = q[0] * p[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3];
    r[1] = q[0] * p[1] + q[1] * p[0] + q[2] * p[3] - q[3] * p[2];
    r[2] = q[0] * p[2] + q[2] * p[0] + q[3] * p[1] - q[1] * p[3];
    r[3] = q[0] * p[3] + q[3] * p[0] + q[1] * p[2] - q[2] * p[1];
}

// Compute quaternions norm.
inline double quat_norm(double q[])
{
    return q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
}

// Compute quaternion conjugate.
inline void quat_conjugate(double q[], double p[])
{
    p[0] = q[0];
    for (uint8_t i = 1; i < 4; ++i)
        p[i] = -q[i];
}

// Compute quaternion inverse.
inline void quat_inverse(double q[], double p[])
{
    double norm = quat_norm(q);
    // p is assigned to q conjugate divided by q norm.
    p[0] = q[0] / norm;
    for (uint8_t i = 1; i < 4; ++i)
        p[i] = -q[i] / norm;
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

#endif // INS_H