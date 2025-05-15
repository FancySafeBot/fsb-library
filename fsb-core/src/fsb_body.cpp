
#include "fsb_types.h"
#include "fsb_quaternion.h"
#include "fsb_motion.h"
#include "fsb_body.h"
#include "fsb_rotation.h"
#include "fsb_linalg3.h"

namespace fsb
{

static Mat3 skew_sq_lt(const Vec3& vec)
{
    const real_t vx2 = vec.x * vec.x;
    const real_t vy2 = vec.y * vec.y;
    const real_t vz2 = vec.z * vec.z;

    return {
        -vy2 - vz2,
        vec.x * vec.y,
        vec.x * vec.z,
        0.0,
        -vx2 - vz2,
        vec.y * vec.z,
        0.0,
        0.0,
        -vx2 - vy2};
}

static Inertia body_rotate_mat3_inertia(const Mat3& rot, const Inertia& inertia)
{
    const real_t c00 = inertia.ixx * rot.m00 + inertia.ixy * rot.m01 + inertia.ixz * rot.m02;
    const real_t c01 = inertia.ixy * rot.m00 + inertia.iyy * rot.m01 + inertia.iyz * rot.m02;
    const real_t c02 = inertia.ixz * rot.m00 + inertia.iyz * rot.m01 + inertia.izz * rot.m02;

    const real_t c10 = inertia.ixx * rot.m10 + inertia.ixy * rot.m11 + inertia.ixz * rot.m12;
    const real_t c11 = inertia.ixy * rot.m10 + inertia.iyy * rot.m11 + inertia.iyz * rot.m12;
    const real_t c12 = inertia.ixz * rot.m10 + inertia.iyz * rot.m11 + inertia.izz * rot.m12;

    const real_t c20 = inertia.ixx * rot.m20 + inertia.ixy * rot.m21 + inertia.ixz * rot.m22;
    const real_t c21 = inertia.ixy * rot.m20 + inertia.iyy * rot.m21 + inertia.iyz * rot.m22;
    const real_t c22 = inertia.ixz * rot.m20 + inertia.iyz * rot.m21 + inertia.izz * rot.m22;

    return {
        rot.m00 * c00 + rot.m01 * c01 + rot.m02 * c02,
        rot.m10 * c10 + rot.m11 * c11 + rot.m12 * c12,
        rot.m20 * c20 + rot.m21 * c21 + rot.m22 * c22,
        rot.m10 * c00 + rot.m11 * c01 + rot.m12 * c02,
        rot.m20 * c00 + rot.m21 * c01 + rot.m22 * c02,
        rot.m20 * c10 + rot.m21 * c11 + rot.m22 * c12};
}

Inertia body_parallel_axis_inertia(const real_t mass, const Vec3& com, const Inertia& inertia)
{
    const Mat3 transl_skew_sq_lt = skew_sq_lt(com);
    return {
        inertia.ixx - mass * transl_skew_sq_lt.m00,
        inertia.iyy - mass * transl_skew_sq_lt.m11,
        inertia.izz - mass * transl_skew_sq_lt.m22,
        inertia.ixy - mass * transl_skew_sq_lt.m10,
        inertia.ixz - mass * transl_skew_sq_lt.m20,
        inertia.iyz - mass * transl_skew_sq_lt.m21};
}

MassProps body_transform_mass_props(const Transform& transf, const MassProps& mass_props)
{
    return {
        mass_props.mass,
        coord_transform_position(transf, mass_props.com),
        body_rotate_inertia(transf.rotation, mass_props.inertia)};
}

Inertia body_rotate_inertia(const Quaternion& rot, const Inertia& inertia)
{
    return body_rotate_mat3_inertia(quat_to_rot(rot), inertia);
}

MassProps body_combine(
    const MassProps& mass_props_a, const Transform& transf_ab, const MassProps& mass_props_b)
{
    MassProps result = {};

    // put mass_props_b in common frame of mass_props_a
    const MassProps mass_props_ab = body_transform_mass_props(transf_ab, mass_props_b);
    // mass and com
    result.mass = mass_props_a.mass + mass_props_ab.mass;
    result.com
        = {(mass_props_a.mass * mass_props_a.com.x + mass_props_ab.mass * mass_props_ab.com.x)
               / result.mass,
           (mass_props_a.mass * mass_props_a.com.y + mass_props_ab.mass * mass_props_ab.com.y)
               / result.mass,
           (mass_props_a.mass * mass_props_a.com.z + mass_props_ab.mass * mass_props_ab.com.z)
               / result.mass};
    // parallel axis inertia a to combined COM
    const Vec3    com_diff_a = vector_subtract(mass_props_a.com, result.com);
    const Inertia inertia_a
        = body_parallel_axis_inertia(mass_props_a.mass, com_diff_a, mass_props_a.inertia);
    // parallel axis inertia b to combined COM
    const Vec3    com_diff_b = vector_subtract(mass_props_ab.com, result.com);
    const Inertia inertia_b
        = body_parallel_axis_inertia(mass_props_ab.mass, com_diff_b, mass_props_ab.inertia);
    // sum at combined COM
    result.inertia
        = {inertia_a.ixx + inertia_b.ixx,
           inertia_a.iyy + inertia_b.iyy,
           inertia_a.izz + inertia_b.izz,
           inertia_a.ixy + inertia_b.ixy,
           inertia_a.ixz + inertia_b.ixz,
           inertia_a.iyz + inertia_b.iyz};
    return result;
}

bool body_inertia_principal_axis(const Inertia& inertia, PrincipalInertia& principal_inertia)
{
    // parameters for eigenvector calculation
    const Mat3Sym inertia_matrix
        = {inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz};

    Vec3       eig_vals = {};
    Vec3       eig_vec0 = {};
    Vec3       eig_vec1 = {};
    Vec3       eig_vec2 = {};
    const bool is_pd = mat3_posdef_symmetric_eigenvectors(
        inertia_matrix, eig_vals, eig_vec0, eig_vec1, eig_vec2);
    if (is_pd)
    {
        // check if the rotation matrix is a proper rotation matrix
        if (const real_t z_dir_test = vector_dot(vector_cross(eig_vec2, eig_vec1), eig_vec0);
            z_dir_test < 0.0)
        {
            // flip the sign of the z-axis
            eig_vec0.x = -eig_vec0.x;
            eig_vec0.y = -eig_vec0.y;
            eig_vec0.z = -eig_vec0.z;
        }
        // result
        const Mat3 inertia_rotation
            = {eig_vec2.x,
               eig_vec2.y,
               eig_vec2.z,
               eig_vec1.x,
               eig_vec1.y,
               eig_vec1.z,
               eig_vec0.x,
               eig_vec0.y,
               eig_vec0.z};
        principal_inertia.rot = rot_to_quat(inertia_rotation);
        principal_inertia.inertia = {eig_vals.z, eig_vals.y, eig_vals.x, 0.0, 0.0, 0.0};
    }

    return is_pd;
}

bool body_validate_inertia_is_pd(const Inertia& inertia)
{
    // parameters for eigenvector calculation
    const Mat3Sym inertia_matrix
        = {inertia.ixx, inertia.iyy, inertia.izz, inertia.ixy, inertia.ixz, inertia.iyz};

    Vec3 eig_vals = {};
    return mat3_posdef_symmetric_eigenvalues(inertia_matrix, eig_vals);
}

Vec3 inertia_multiply_vector(const Inertia& inertia, const Vec3& vec)
{
    return {
        inertia.ixx * vec.x + inertia.ixy * vec.y + inertia.ixz * vec.z,
        inertia.ixy * vec.x + inertia.iyy * vec.y + inertia.iyz * vec.z,
        inertia.ixz * vec.x + inertia.iyz * vec.y + inertia.izz * vec.z
    };
}

Vec3 inertia_cross_multiply_vector(const Inertia& inertia, const Vec3& vel)
{
    const real_t x0 = inertia.ixy * vel.z;
    const real_t x1 = inertia.ixz * vel.y;
    const real_t x2 = -inertia.iyz * vel.x;
    return {
        -vel.x * (x0 - x1) - vel.y * (inertia.iyy * vel.z - inertia.iyz * vel.y)
            - vel.z * (inertia.iyz * vel.z - inertia.izz * vel.y),
        vel.x * (inertia.ixx * vel.z - inertia.ixz * vel.x) + vel.y * (x0 + x2)
            + vel.z * (inertia.ixz * vel.z - inertia.izz * vel.x),
        -vel.x * (inertia.ixx * vel.y - inertia.ixy * vel.x)
            - vel.y * (inertia.ixy * vel.y - inertia.iyy * vel.x) - vel.z * (x1 + x2)};
}

} // namespace fsb
