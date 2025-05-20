
#include "fsb_quaternion.h"
#include "fsb_types.h"
#include <cmath>

namespace fsb
{

Quaternion quat_multiply(const Quaternion& q_a, const Quaternion& q_b)
{
    return {
        q_a.qw * q_b.qw - q_a.qx * q_b.qx - q_a.qy * q_b.qy - q_a.qz * q_b.qz,
        q_a.qw * q_b.qx + q_a.qx * q_b.qw + q_a.qy * q_b.qz - q_a.qz * q_b.qy,
        q_a.qw * q_b.qy + q_a.qy * q_b.qw - q_a.qx * q_b.qz + q_a.qz * q_b.qx,
        q_a.qw * q_b.qz + q_a.qx * q_b.qy - q_a.qy * q_b.qx + q_a.qz * q_b.qw};
}

Vec3 quat_rotate_vector(const Quaternion& q_in, const Vec3& p_in)
{
    const real_t q02 = q_in.qw * q_in.qw;
    const real_t q12 = q_in.qx * q_in.qx;
    const real_t q22 = q_in.qy * q_in.qy;
    const real_t q32 = q_in.qz * q_in.qz;

    return {
        p_in.x * q02 + p_in.x * q12 - p_in.x * q22 - p_in.x * q32 - p_in.y * q_in.qw * q_in.qz * 2.0
            + p_in.y * q_in.qx * q_in.qy * 2.0 + p_in.z * q_in.qw * q_in.qy * 2.0 + p_in.z * q_in.qx * q_in.qz * 2.0,
        p_in.y * q02 - p_in.y * q12 + p_in.y * q22 - p_in.y * q32 + p_in.x * q_in.qw * q_in.qz * 2.0
            + p_in.x * q_in.qx * q_in.qy * 2.0 - p_in.z * q_in.qw * q_in.qx * 2.0 + p_in.z * q_in.qy * q_in.qz * 2.0,
        p_in.z * q02 - p_in.z * q12 - p_in.z * q22 + p_in.z * q32 - p_in.x * q_in.qw * q_in.qy * 2.0
            + p_in.y * q_in.qw * q_in.qx * 2.0 + p_in.x * q_in.qx * q_in.qz * 2.0 + p_in.y * q_in.qy * q_in.qz * 2.0};
}

Quaternion quat_conjugate(const Quaternion& q_in)
{
    return {q_in.qw, -q_in.qx, -q_in.qy, -q_in.qz};
}

real_t quat_norm(const Quaternion& q_in)
{
    return sqrt((q_in.qw * q_in.qw) + (q_in.qx * q_in.qx) + (q_in.qy * q_in.qy) + (q_in.qz * q_in.qz));
}

void quat_normalize(Quaternion& q_inout)
{
    if (const real_t q_norm = quat_norm(q_inout);
        q_norm < FSB_TOL)
    {
        q_inout.qw = 1.0;
        q_inout.qx = 0.0;
        q_inout.qy = 0.0;
        q_inout.qz = 0.0;
    }
    else
    {
        q_inout.qw /= q_norm;
        q_inout.qx /= q_norm;
        q_inout.qy /= q_norm;
        q_inout.qz /= q_norm;
    }
    if (q_inout.qw < 0.0)
    {
        q_inout.qw = -q_inout.qw;
        q_inout.qx = -q_inout.qx;
        q_inout.qy = -q_inout.qy;
        q_inout.qz = -q_inout.qz;
    }
}

Vec3 quat_log(const Quaternion& q_in)
{
    const real_t p_norm = sqrt((q_in.qx * q_in.qx) + (q_in.qy * q_in.qy) + (q_in.qz * q_in.qz));

    Vec3 p_out = {0.0, 0.0, 0.0};
    if (p_norm >= FSB_TOL)
    {
        const real_t phi_norm = acos(q_in.qw) / p_norm;
        p_out.x = phi_norm * q_in.qx;
        p_out.y = phi_norm * q_in.qy;
        p_out.z = phi_norm * q_in.qz;
    }
    return p_out;
}

Quaternion quat_exp(const Vec3& v_in)
{
    const real_t p_norm = sqrt((v_in.x * v_in.x) + (v_in.y * v_in.y) + (v_in.z * v_in.z));
    const real_t sp_norm = sin(p_norm);

    Quaternion q_out = {1.0, 0.0, 0.0, 0.0};
    if (p_norm >= FSB_TOL)
    {
        q_out.qw = cos(p_norm);
        q_out.qx = v_in.x * sp_norm / p_norm;
        q_out.qy = v_in.y * sp_norm / p_norm;
        q_out.qz = v_in.z * sp_norm / p_norm;
    }
    return q_out;
}

Quaternion quat_boxplus(const Quaternion& quat, const Vec3& phi_delta)
{
    // work
    const Vec3 phi_half = {phi_delta.x / 2.0, phi_delta.y / 2.0, phi_delta.z / 2.0};
    return quat_multiply(quat, quat_exp(phi_half));
}

Vec3 quat_boxminus(const Quaternion& q_a, const Quaternion& q_b)
{
    Vec3 phi_delta = quat_log(quat_multiply(quat_conjugate(q_b), q_a));
    phi_delta.x = 2.0 * phi_delta.x;
    phi_delta.y = 2.0 * phi_delta.y;
    phi_delta.z = 2.0 * phi_delta.z;
    return phi_delta;
}

Quaternion quat_circplus(const Quaternion& quat, const Vec3& phi_delta)
{
    // work
    const Vec3 phi_half = {phi_delta.x / 2.0, phi_delta.y / 2.0, phi_delta.z / 2.0};
    return quat_multiply(quat_exp(phi_half), quat);
}

Vec3 quat_circminus(const Quaternion& q_a, const Quaternion& q_b)
{
    Vec3 phi_delta = quat_log(quat_multiply(q_a, quat_conjugate(q_b)));
    phi_delta.x = 2.0 * phi_delta.x;
    phi_delta.y = 2.0 * phi_delta.y;
    phi_delta.z = 2.0 * phi_delta.z;
    return phi_delta;
}

Quaternion quat_identity()
{
    return {1.0, 0.0, 0.0, 0.0};
}

Quaternion quat_rx(const real_t rx)
{
    return {cos(rx / 2.0), sin(rx / 2.0), 0.0, 0.0};
}
Quaternion quat_ry(const real_t ry)
{
    return {cos(ry / 2.0), 0.0, sin(ry / 2.0), 0.0};
}
Quaternion quat_rz(const real_t rz)
{
    return {cos(rz / 2.0), 0.0, 0.0, sin(rz / 2.0)};
}

} // namespace fsb
