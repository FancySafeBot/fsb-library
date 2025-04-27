
#include "fsb_motion.h"
#include "fsb_quaternion.h"
#include "fsb_rotation.h"
#include "fsb_types.h"

#include <cmath>

namespace fsb
{

Transform transform_identity()
{
    return {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
    };
}

Transform transform_inverse(const Transform& transf)
{
    const Quaternion quat_conj = quat_conjugate(transf.rotation);
    const Vec3       work = quat_rotate_vector(quat_conj, transf.translation);
    return {
        quat_conj,
        {-work.x, -work.y, -work.z},
    };
}

Transform coord_transform(const Transform& transf_a, const Transform& transf_b)
{
    const Vec3 work = quat_rotate_vector(transf_a.rotation, transf_b.translation);
    Transform  result = {
        quat_multiply(transf_a.rotation, transf_b.rotation),
        {work.x + transf_a.translation.x,
                                                work.y + transf_a.translation.y,
                                                work.z + transf_a.translation.z}
    };
    if (result.rotation.qw < 0.0)
    {
        result.rotation.qw = -result.rotation.qw;
        result.rotation.qx = -result.rotation.qx;
        result.rotation.qy = -result.rotation.qy;
        result.rotation.qz = -result.rotation.qz;
    }
    return result;
}

Vec3 coord_transform_position(const Transform& transf, const Vec3& pos)
{
    const Vec3 work = quat_rotate_vector(transf.rotation, pos);
    return {
        work.x + transf.translation.x,
        work.y + transf.translation.y,
        work.z + transf.translation.z};
}

Transform coord_transform_apply_space_offset(const MotionVector& offset, const Transform& transf)
{
    const Transform offset_transform = {rotvec_to_quat(offset.angular), offset.linear};
    return coord_transform(offset_transform, transf);
}

MotionVector coord_transform_get_space_offset(const Transform& transf_post, const Transform& transf)
{
    const Transform offset_transform = coord_transform(transf, transform_inverse(transf_post));
    return {quat_to_rotvec(offset_transform.rotation), offset_transform.translation};
}

Transform
coord_transform_apply_body_offset(const Transform& transf_initial, const MotionVector& offset)
{
    return {
        quat_boxplus(transf_initial.rotation, offset.angular),
        vector_add(transf_initial.translation, offset.linear)};
}

MotionVector
coord_transform_get_body_offset(const Transform& transf_initial, const Transform& transf_final)
{
    return {
        quat_boxminus(transf_initial.rotation, transf_final.rotation),
        vector_subtract(transf_final.translation, transf_initial.translation)};
}

MotionVector motion_transform_velocity(
    const Transform& parent_transf, const MotionVector& parent_velocity,
    const Transform& motion_transf, const MotionVector& motion_velocity)
{
    const Vec3 ang1 = quat_rotate_vector(parent_transf.rotation, motion_velocity.angular);

    const Vec3 lin1a = vector_cross(
        parent_velocity.angular,
        quat_rotate_vector(parent_transf.rotation, motion_transf.translation));
    const Vec3 lin1b = quat_rotate_vector(parent_transf.rotation, motion_velocity.linear);
    const Vec3 lin1 = vector_add(lin1a, lin1b);

    return {vector_add(parent_velocity.angular, ang1), vector_add(parent_velocity.linear, lin1)};
}

MotionVector motion_transform_acceleration(
    const Transform& parent_transf, const MotionVector& parent_velocity,
    const MotionVector& parent_acceleration, const Transform& motion_transf,
    const MotionVector& motion_velocity, const MotionVector& motion_acceleration)
{
    const Vec3 ang_a = vector_cross(
        parent_velocity.angular,
        quat_rotate_vector(parent_transf.rotation, motion_velocity.angular));
    const Vec3 ang_b = quat_rotate_vector(parent_transf.rotation, motion_acceleration.angular);
    const Vec3 ang_sum = vector_add(ang_a, ang_b);

    const Vec3 pos_space = quat_rotate_vector(parent_transf.rotation, motion_transf.translation);
    const Vec3 lin_a = vector_cross(parent_acceleration.angular, pos_space);
    const Vec3 lin_b
        = vector_cross(parent_velocity.angular, vector_cross(parent_velocity.angular, pos_space));
    const Vec3 lin_c = vector_scale(
        2.0,
        vector_cross(
            parent_velocity.angular,
            quat_rotate_vector(parent_transf.rotation, motion_velocity.linear)));
    const Vec3 lin_d = quat_rotate_vector(parent_transf.rotation, motion_acceleration.linear);
    const Vec3 lin_sum
        = {lin_a.x + lin_b.x + lin_c.x + lin_d.x,
           lin_a.y + lin_b.y + lin_c.y + lin_d.y,
           lin_a.z + lin_b.z + lin_c.z + lin_d.z};

    return {
        vector_add(parent_acceleration.angular, ang_sum),
        vector_add(parent_acceleration.linear, lin_sum)};
}

MotionVector motion_transform_velocity_position(
    const Transform& parent_transf, const MotionVector& parent_velocity, const Vec3& child_position)
{
    const Vec3 lin = vector_cross(
        parent_velocity.angular, quat_rotate_vector(parent_transf.rotation, child_position));
    return {parent_velocity.angular, vector_add(parent_velocity.linear, lin)};
}

MotionVector motion_transform_acceleration_position(
    const Transform& parent_transf, const MotionVector& parent_velocity,
    const MotionVector& parent_acceleration, const Vec3& child_position)
{
    const Vec3 pos_space = quat_rotate_vector(parent_transf.rotation, child_position);
    const Vec3 lin_a = vector_cross(parent_acceleration.angular, pos_space);
    const Vec3 lin_b
        = vector_cross(parent_velocity.angular, vector_cross(parent_velocity.angular, pos_space));
    const Vec3 lin_sum = vector_add(lin_a, lin_b);

    return {parent_acceleration.angular, vector_add(parent_acceleration.linear, lin_sum)};
}

MotionVector
motion_transform_space_to_body_velocity(const Transform& pose, const MotionVector& space_velocity)
{
    const Mat3 rot = quat_to_rot(pose.rotation);
    return {
        rotate_mat3_transpose(rot, space_velocity.angular),
        rotate_mat3_transpose(rot, space_velocity.linear)};
}

MotionVector
motion_transform_body_to_space_velocity(const Transform& pose, const MotionVector& body_velocity)
{
    const Mat3 rot = quat_to_rot(pose.rotation);
    return {rotate_mat3(rot, body_velocity.angular), rotate_mat3(rot, body_velocity.linear)};
}

Vec3 vector_add(const Vec3& v_a, const Vec3& v_b)
{
    return {v_a.x + v_b.x, v_a.y + v_b.y, v_a.z + v_b.z};
}

Vec3 vector_subtract(const Vec3& v_a, const Vec3& v_b)
{
    return {v_a.x - v_b.x, v_a.y - v_b.y, v_a.z - v_b.z};
}

Vec3 vector_scale(const real_t scalar, const Vec3& vec)
{
    return {vec.x * scalar, vec.y * scalar, vec.z * scalar};
}

Vec3 vector_cross(const Vec3& v_a, const Vec3& v_b)
{
    return {
        v_a.y * v_b.z - v_a.z * v_b.y,
        v_a.z * v_b.x - v_a.x * v_b.z,
        v_a.x * v_b.y - v_a.y * v_b.x};
}

real_t vector_dot(const Vec3& v_a, const Vec3& v_b)
{
    return v_a.x * v_b.x + v_a.y * v_b.y + v_a.z * v_b.z;
}

real_t vector_norm(const Vec3& vec)
{
    return sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
}

//
// MotionVector motion_transform_body_velocity(
//     const Transform& parent_transf, const MotionVector& parent_velocity, const Transform&
//     motion_transf, const MotionVector& motion_velocity)
// {
//     const Vec3 ang = quat_rotate_vector(quat_conjugate(motion_transf.rotation),
//     parent_velocity.angular);
//
//     const Vec3 lin1a = vector_cross(parent_velocity.angular, motion_transf.translation);
//     const Vec3 lin1b = {lin1a.x + motion_velocity.linear.x, lin1a.y + motion_velocity.linear.y,
//     lin1a.z + motion_velocity.linear.z}; const Vec3 lin1 =
//     quat_rotate_vector(parent_transf.rotation, lin1b);
//
//     return {
//         vector_add(motion_velocity.angular, ang),
//         vector_add(parent_velocity.linear, lin1)
//     };
// }
//
// MotionVector motion_transform_body_acceleration(
//     const Transform& parent_transf, const MotionVector& parent_velocity, const MotionVector&
//     parent_acceleration, const Transform& motion_transf, const MotionVector& motion_velocity,
//     const MotionVector& motion_acceleration)
// {
//     const Vec3 ang1 = quat_rotate_vector(quat_conjugate(motion_transf.rotation),
//     parent_acceleration.angular); const Vec3 ang2 =
//     vector_cross(quat_rotate_vector(quat_conjugate(motion_transf.rotation),
//     parent_velocity.angular), motion_velocity.angular);
//
//     const Vec3 lin1a = vector_cross(parent_acceleration.angular, motion_transf.translation);
//     const Vec3 lin1b = vector_cross(parent_velocity.angular,
//     vector_cross(parent_velocity.angular, motion_transf.translation)); const Vec3 lin1c =
//     vector_cross(parent_velocity.angular, motion_velocity.linear); const Vec3 lin1d
//         = {lin1a.x + lin1b.x + 2.0 * lin1c.x + motion_acceleration.linear.x, lin1a.y + lin1b.y
//         + 2.0 * lin1c.y + motion_acceleration.linear.y,
//            lin1a.z + lin1b.z + 2.0 * lin1c.z + motion_acceleration.linear.z};
//     const Vec3 lin1 = quat_rotate_vector(parent_transf.rotation, lin1d);
//
//     return {
//         {motion_acceleration.angular.x + ang1.x + ang2.x, motion_acceleration.angular.y + ang1.y
//         + ang2.y,
//          motion_acceleration.angular.z + ang1.z + ang2.z },
//         {parent_acceleration.linear.x + lin1.x,          parent_acceleration.linear.y + lin1.y,
//         parent_acceleration.linear.z + lin1.z}
//     };
// }
//

} // namespace fsb
