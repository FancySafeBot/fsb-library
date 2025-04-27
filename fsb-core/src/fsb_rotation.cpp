
#include <cmath>
#include "fsb_types.h"
#include "fsb_quaternion.h"
#include "fsb_rotation.h"

namespace fsb
{

Mat3 skew_symmetric(const Vec3& vec)
{
    return {0.0, vec.z, -vec.y, -vec.z, 0.0, vec.x, vec.y, -vec.x, 0.0};
}

Mat3 rot_identity()
{
    return {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

Mat3 rot_rx(const real_t rx)
{
    const real_t sx = sin(rx);
    const real_t cx = cos(rx);
    return {1.0, 0.0, 0.0, 0.0, cx, sx, 0.0, -sx, cx};
}

Mat3 rot_ry(const real_t ry)
{
    const real_t sy = sin(ry);
    const real_t cy = cos(ry);
    return {cy, 0.0, -sy, 0.0, 1.0, 0.0, sy, 0.0, cy};
}

Mat3 rot_rz(const real_t rz)
{
    const real_t sz = sin(rz);
    const real_t cz = cos(rz);
    return {cz, sz, 0.0, -sz, cz, 0.0, 0.0, 0.0, 1.0};
}

Vec3 quat_to_ezyx(const Quaternion& quat)
{
    Vec3 ezyx = {0.0, 0.0, 0.0};

    const real_t     sval = 2.0 * (quat.qw * quat.qy - quat.qx * quat.qz);
    if (constexpr real_t unit_tol = (1.0 - FSB_TOL);
        sval < -unit_tol)
    {
        ezyx.z = 0.0;
        ezyx.y = -M_PI_2;
        ezyx.x = 2.0 * atan2(quat.qx, -quat.qy);
    }
    else if (sval > unit_tol)
    {
        ezyx.z = 0.0;
        ezyx.y = M_PI_2;
        ezyx.x = 2.0 * atan2(-quat.qx, quat.qy);
    }
    else
    {
        const real_t sqw = quat.qw * quat.qw;
        const real_t sqx = quat.qx * quat.qx;
        const real_t sqy = quat.qy * quat.qy;
        const real_t sqz = quat.qz * quat.qz;
        ezyx.z = atan2(2.0 * (quat.qy * quat.qz + quat.qw * quat.qx), sqw - sqx - sqy + sqz);
        ezyx.y = asin(sval);
        ezyx.x = atan2(2.0 * (quat.qx * quat.qy + quat.qw * quat.qz), sqw + sqx - sqy - sqz);
    }

    return ezyx;
}

Quaternion ezyx_to_quat(const Vec3& ezyx)
{
    const real_t ex_2 = ezyx.z / 2.0;
    const real_t ey_2 = ezyx.y / 2.0;
    const real_t ez_2 = ezyx.x / 2.0;

    return {
        cos(ex_2) * cos(ey_2) * cos(ez_2) + sin(ex_2) * sin(ey_2) * sin(ez_2),
        sin(ex_2) * cos(ey_2) * cos(ez_2) - cos(ex_2) * sin(ey_2) * sin(ez_2),
        cos(ex_2) * sin(ey_2) * cos(ez_2) + sin(ex_2) * cos(ey_2) * sin(ez_2),
        cos(ex_2) * cos(ey_2) * sin(ez_2) - sin(ex_2) * sin(ey_2) * cos(ez_2)};
}

Mat3 quat_to_rot(const Quaternion& quat)
{
    /* work */
    const real_t q02_2 = quat.qw * quat.qy * 2.0;
    const real_t q12_2 = quat.qx * quat.qy * 2.0;
    const real_t q13_2 = quat.qx * quat.qz * 2.0;
    const real_t q23_2 = quat.qy * quat.qz * 2.0;
    const real_t q11 = quat.qx * quat.qx;
    const real_t q22 = quat.qy * quat.qy;
    const real_t q33 = quat.qz * quat.qz;

    return {
        -q33 * 2.0 - q22 * 2.0 + 1.0, q12_2 + quat.qw * quat.qz * 2.0, -q02_2 + q13_2, q12_2 - quat.qw * quat.qz * 2.0,
        -q33 * 2.0 - q11 * 2.0 + 1.0, q23_2 + quat.qw * quat.qx * 2.0, q02_2 + q13_2,  q23_2 - quat.qw * quat.qx * 2.0,
        -q11 * 2.0 - q22 * 2.0 + 1.0};
}

Quaternion rot_to_quat(const Mat3& rot)
{
    Quaternion   quat = {1.0, 0.0, 0.0, 0.0};
    if (const real_t trace = rot.m00 + rot.m11 + rot.m22;
        trace > 0.0)
    {
        // quaternion element |qw| greater than 1/2
        const real_t sqw = 2.0 * sqrt(trace + 1.0); // 4 * qw
        quat.qw = 0.25 * sqw;
        quat.qx = (rot.m21 - rot.m12) / sqw;
        quat.qy = (rot.m02 - rot.m20) / sqw;
        quat.qz = (rot.m10 - rot.m01) / sqw;
    }
    else if ((rot.m00 > rot.m11) && (rot.m00 > rot.m22))
    {
        // sqx = 4 * qx
        const real_t sqx = 2.0 * sqrt(1.0 + rot.m00 - rot.m11 - rot.m22);
        quat.qw = (rot.m21 - rot.m12) / sqx;
        quat.qx = 0.25 * sqx;
        quat.qy = (rot.m01 + rot.m10) / sqx;
        quat.qz = (rot.m02 + rot.m20) / sqx;
    }
    else if (rot.m11 > rot.m22)
    {
        // sqw = 4 * qy
        const real_t sqy = 2.0 * sqrt(1.0 + rot.m11 - rot.m00 - rot.m22);
        quat.qw = (rot.m02 - rot.m20) / sqy;
        quat.qx = (rot.m01 + rot.m10) / sqy;
        quat.qy = 0.25 * sqy;
        quat.qz = (rot.m12 + rot.m21) / sqy;
    }
    else
    {
        // sqz = 4 * qz
        const real_t sqz = 2.0 * sqrt(1.0 + rot.m22 - rot.m00 - rot.m11);
        quat.qw = (rot.m10 - rot.m01) / sqz;
        quat.qx = (rot.m02 + rot.m20) / sqz;
        quat.qy = (rot.m12 + rot.m21) / sqz;
        quat.qz = 0.25 * sqz;
    }

    if (quat.qw < 0.0)
    {
        quat.qw = -quat.qw;
        quat.qx = -quat.qx;
        quat.qy = -quat.qy;
        quat.qz = -quat.qz;
    }
    return quat;
}

Vec3 quat_to_rotvec(const Quaternion& quat)
{
    Vec3 phi = quat_log(quat);
    phi.x = 2.0 * phi.x;
    phi.y = 2.0 * phi.y;
    phi.z = 2.0 * phi.z;
    return phi;
}

Quaternion rotvec_to_quat(const Vec3& rotvec)
{
    const Vec3 phi_half = {rotvec.x / 2.0, rotvec.y / 2.0, rotvec.z / 2.0};
    return quat_exp(phi_half);
}

Vec3 rotate_mat3(const Mat3& rot, const Vec3& vec)
{
    return {
        rot.m00 * vec.x + rot.m01 * vec.y + rot.m02 * vec.z,
        rot.m10 * vec.x + rot.m11 * vec.y + rot.m12 * vec.z,
        rot.m20 * vec.x + rot.m21 * vec.y + rot.m22 * vec.z
    };
}

Vec3 rotate_mat3_transpose(const Mat3& rot, const Vec3& vec)
{
    return {
        rot.m00 * vec.x + rot.m10 * vec.y + rot.m20 * vec.z,
        rot.m01 * vec.x + rot.m11 * vec.y + rot.m21 * vec.z,
        rot.m02 * vec.x + rot.m12 * vec.y + rot.m22 * vec.z
    };
}

} // namespace fsb
