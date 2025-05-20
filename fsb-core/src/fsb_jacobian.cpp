
#include <cstddef>
#include "fsb_configuration.h"
#include "fsb_types.h"
#include "fsb_quaternion.h"
#include "fsb_motion.h"
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_joint.h"
#include "fsb_jacobian.h"
#include "fsb_rotation.h"

namespace fsb
{

static Mat3 pos_skew_rot(const Vec3& pos, const Mat3 & rot)
{
    // mat = skew(pos) * rot
    return {
        pos.y * rot.m20 - pos.z * rot.m10,
        -pos.x * rot.m20 + pos.z * rot.m00,
        pos.x * rot.m10 - pos.y * rot.m00,
        pos.y * rot.m21 - pos.z * rot.m11,
        -pos.x * rot.m21 + pos.z * rot.m01,
        pos.x * rot.m11 - pos.y * rot.m01,
        pos.y * rot.m22 - pos.z * rot.m12,
        -pos.x * rot.m22 + pos.z * rot.m02,
        pos.x * rot.m12 - pos.y * rot.m02
    };
}

static Vec3 quat_to_rotx(const Quaternion& quat)
{
    return {
        -2.0 * quat.qy * quat.qy - 2.0 * quat.qz * quat.qz + 1.0,
        2.0 * quat.qw * quat.qz + 2.0 * quat.qx * quat.qy,
        -2.0 * quat.qw * quat.qy + 2.0 * quat.qx * quat.qz};
}

static Vec3 quat_to_roty(const Quaternion& quat)
{
    return {
        -2.0 * quat.qw * quat.qz + 2.0 * quat.qx * quat.qy,
        -2.0 * quat.qx * quat.qx - 2.0 * quat.qz * quat.qz + 1.0,
        2.0 * quat.qw * quat.qx + 2.0 * quat.qy * quat.qz};
}

static Vec3 quat_to_rotz(const Quaternion& quat)
{
    return {
            2.0 * quat.qw * quat.qy + 2.0 * quat.qx * quat.qz,
            -2.0 * quat.qw * quat.qx + 2.0 * quat.qy * quat.qz,
            -2.0 * quat.qx * quat.qx - 2.0 * quat.qy * quat.qy + 1.0};
}

static void calculate_jacobian_joint_columns(
    const Joint& joint, const Transform& body_base_pose, const Transform& target_pose,
    Jacobian& jacobian)
{
    if (joint.type == JointType::REVOLUTE_X)
    {
        const size_t jac_col = joint.dof_index;

        // rotation component
        const Vec3 rot_x = quat_to_rotx(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col)] = rot_x.x;
        jacobian.j[jacobian_index(1U, jac_col)] = rot_x.y;
        jacobian.j[jacobian_index(2U, jac_col)] = rot_x.z;

        // translation component
        const Vec3 pos_skew_x = vector_cross(vector_subtract(body_base_pose.translation, target_pose.translation), rot_x);
        jacobian.j[jacobian_index(3U, jac_col)] = pos_skew_x.x;
        jacobian.j[jacobian_index(4U, jac_col)] = pos_skew_x.y;
        jacobian.j[jacobian_index(5U, jac_col)] = pos_skew_x.z;
    }
    else if (joint.type == JointType::REVOLUTE_Y)
    {
        const size_t jac_col = joint.dof_index;

        // rotation component
        const Vec3 rot_y = quat_to_roty(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col)] = rot_y.x;
        jacobian.j[jacobian_index(1U, jac_col)] = rot_y.y;
        jacobian.j[jacobian_index(2U, jac_col)] = rot_y.z;

        // translation component
        const Vec3 pos_skew_y = vector_cross(vector_subtract(body_base_pose.translation, target_pose.translation), rot_y);
        jacobian.j[jacobian_index(3U, jac_col)] = pos_skew_y.x;
        jacobian.j[jacobian_index(4U, jac_col)] = pos_skew_y.y;
        jacobian.j[jacobian_index(5U, jac_col)] = pos_skew_y.z;
    }
    else if (joint.type == JointType::REVOLUTE_Z)
    {
        const size_t jac_col = joint.dof_index;

        // rotation component
        const Vec3 rot_z = quat_to_rotz(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col)] = rot_z.x;
        jacobian.j[jacobian_index(1U, jac_col)] = rot_z.y;
        jacobian.j[jacobian_index(2U, jac_col)] = rot_z.z;

        // translation component
        const Vec3 pos_skew_z = vector_cross(vector_subtract(body_base_pose.translation, target_pose.translation), rot_z);
        jacobian.j[jacobian_index(3U, jac_col)] = pos_skew_z.x;
        jacobian.j[jacobian_index(4U, jac_col)] = pos_skew_z.y;
        jacobian.j[jacobian_index(5U, jac_col)] = pos_skew_z.z;
    }
    else if (joint.type == JointType::PRISMATIC_X)
    {
        const size_t jac_col = joint.dof_index;

        // translation component
        const Vec3 rot_x = quat_to_rotx(body_base_pose.rotation);
        jacobian.j[jacobian_index(3U, jac_col)] = rot_x.x;
        jacobian.j[jacobian_index(4U, jac_col)] = rot_x.y;
        jacobian.j[jacobian_index(5U, jac_col)] = rot_x.z;
    }
    else if (joint.type == JointType::PRISMATIC_Y)
    {
        const size_t jac_col = joint.dof_index;

        // translation component
        const Vec3 rot_y = quat_to_roty(body_base_pose.rotation);
        jacobian.j[jacobian_index(3U, jac_col)] = rot_y.x;
        jacobian.j[jacobian_index(4U, jac_col)] = rot_y.y;
        jacobian.j[jacobian_index(5U, jac_col)] = rot_y.z;
    }
    else if (joint.type == JointType::PRISMATIC_Z)
    {
        const size_t jac_col = joint.dof_index;

        // translation component
        const Vec3 rot_z = quat_to_rotz(body_base_pose.rotation);
        jacobian.j[jacobian_index(3U, jac_col)] = rot_z.x;
        jacobian.j[jacobian_index(4U, jac_col)] = rot_z.y;
        jacobian.j[jacobian_index(5U, jac_col)] = rot_z.z;
    }
    else if (joint.type == JointType::SPHERICAL)
    {
        const size_t jac_col0 = joint.dof_index;
        const size_t jac_col1 = joint.dof_index + 1U;
        const size_t jac_col2 = joint.dof_index + 2U;

        // jac: angular
        const Mat3 body_rot = quat_to_rot(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col0)] = body_rot.m00;
        jacobian.j[jacobian_index(1U, jac_col0)] = body_rot.m10;
        jacobian.j[jacobian_index(2U, jac_col0)] = body_rot.m20;
        jacobian.j[jacobian_index(0U, jac_col1)] = body_rot.m01;
        jacobian.j[jacobian_index(1U, jac_col1)] = body_rot.m11;
        jacobian.j[jacobian_index(2U, jac_col1)] = body_rot.m21;
        jacobian.j[jacobian_index(0U, jac_col2)] = body_rot.m02;
        jacobian.j[jacobian_index(1U, jac_col2)] = body_rot.m12;
        jacobian.j[jacobian_index(2U, jac_col2)] = body_rot.m22;

        // jac: linear
        const Mat3 skew_rot = pos_skew_rot(vector_subtract(body_base_pose.translation, target_pose.translation), body_rot);
        jacobian.j[jacobian_index(3U, jac_col0)] = skew_rot.m00;
        jacobian.j[jacobian_index(4U, jac_col0)] = skew_rot.m10;
        jacobian.j[jacobian_index(5U, jac_col0)] = skew_rot.m20;
        jacobian.j[jacobian_index(3U, jac_col1)] = skew_rot.m01;
        jacobian.j[jacobian_index(4U, jac_col1)] = skew_rot.m11;
        jacobian.j[jacobian_index(5U, jac_col1)] = skew_rot.m21;
        jacobian.j[jacobian_index(3U, jac_col2)] = skew_rot.m02;
        jacobian.j[jacobian_index(4U, jac_col2)] = skew_rot.m12;
        jacobian.j[jacobian_index(5U, jac_col2)] = skew_rot.m22;
    }
    else if (joint.type == JointType::CARTESIAN)
    {
        const size_t jac_col0 = joint.dof_index;
        const size_t jac_col1 = joint.dof_index + 1U;
        const size_t jac_col2 = joint.dof_index + 2U;
        const size_t jac_col3 = joint.dof_index + 3U;
        const size_t jac_col4 = joint.dof_index + 4U;
        const size_t jac_col5 = joint.dof_index + 5U;

        // jac sub-matrix: linear
        const Mat3 body_rot = quat_to_rot(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col0)] = body_rot.m00;
        jacobian.j[jacobian_index(1U, jac_col0)] = body_rot.m10;
        jacobian.j[jacobian_index(2U, jac_col0)] = body_rot.m20;
        jacobian.j[jacobian_index(0U, jac_col1)] = body_rot.m01;
        jacobian.j[jacobian_index(1U, jac_col1)] = body_rot.m11;
        jacobian.j[jacobian_index(2U, jac_col1)] = body_rot.m21;
        jacobian.j[jacobian_index(0U, jac_col2)] = body_rot.m02;
        jacobian.j[jacobian_index(1U, jac_col2)] = body_rot.m12;
        jacobian.j[jacobian_index(2U, jac_col2)] = body_rot.m22;

        // jac sub-matrix: angular
        const Mat3 skew_rot = pos_skew_rot(vector_subtract(body_base_pose.translation, target_pose.translation), body_rot);
        jacobian.j[jacobian_index(3U, jac_col0)] = skew_rot.m00;
        jacobian.j[jacobian_index(4U, jac_col0)] = skew_rot.m10;
        jacobian.j[jacobian_index(5U, jac_col0)] = skew_rot.m20;
        jacobian.j[jacobian_index(3U, jac_col1)] = skew_rot.m01;
        jacobian.j[jacobian_index(4U, jac_col1)] = skew_rot.m11;
        jacobian.j[jacobian_index(5U, jac_col1)] = skew_rot.m21;
        jacobian.j[jacobian_index(3U, jac_col2)] = skew_rot.m02;
        jacobian.j[jacobian_index(4U, jac_col2)] = skew_rot.m12;
        jacobian.j[jacobian_index(5U, jac_col2)] = skew_rot.m22;

        jacobian.j[jacobian_index(3U, jac_col3)] = body_rot.m00;
        jacobian.j[jacobian_index(4U, jac_col3)] = body_rot.m10;
        jacobian.j[jacobian_index(5U, jac_col3)] = body_rot.m20;
        jacobian.j[jacobian_index(3U, jac_col4)] = body_rot.m01;
        jacobian.j[jacobian_index(4U, jac_col4)] = body_rot.m11;
        jacobian.j[jacobian_index(5U, jac_col4)] = body_rot.m21;
        jacobian.j[jacobian_index(3U, jac_col5)] = body_rot.m02;
        jacobian.j[jacobian_index(4U, jac_col5)] = body_rot.m12;
        jacobian.j[jacobian_index(5U, jac_col5)] = body_rot.m22;
    }
    else
    {
        // nothing to do, leave jacobian columns at zero
    }
}

JacobianError calculate_jacobian(
    size_t body_index, const BodyTree& body_tree,
    const BodyCartesianPva& cartesian_pva, Jacobian& jacobian)
{
    // initialize
    jacobian = {};
    auto result = JacobianError::SUCCESS;

    // check if body index is valid
    if (body_index >= body_tree.get_num_bodies())
    {
        result = JacobianError::BODY_NOT_IN_TREE;
    }
    else if (body_index > 0U)
    {
        const Transform& target_pose = cartesian_pva.body[body_index].pose;
        // propagate through parent bodies
        while (body_index > 0U)
        {
            auto err = BodyTreeError::SUCCESS;
            // current body and joint
            const Body& body = body_tree.get_body(body_index, err);
            // no error since body_index < num_bodies
            const Joint& joint = body_tree.get_joint(body.joint_index, err);
            // body base pose
            const Transform& parent_pose = cartesian_pva.body[joint.parent_body_index].pose;
            const Transform body_base_pose = coord_transform(parent_pose, joint.parent_joint_transform);
            // compute jacobian column
            calculate_jacobian_joint_columns(joint, body_base_pose, target_pose, jacobian);
            // next body
            body_index = joint.parent_body_index;
        }
    }
    else
    {
        // no joints: all zero jacobian
    }
    return result;
}

MotionVector jacobian_multiply(
    const Jacobian& jacobian, const JointSpace& joint_motion, size_t dofs)
{
    MotionVector result = {};
    if (dofs > MaxSize::dofs)
    {
        dofs = MaxSize::dofs;
    }
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        result.angular.x += jacobian.j[jacobian_index(0U, ind)] * joint_motion.qv[ind];
        result.angular.y += jacobian.j[jacobian_index(1U, ind)] * joint_motion.qv[ind];
        result.angular.z += jacobian.j[jacobian_index(2U, ind)] * joint_motion.qv[ind];
        result.linear.x += jacobian.j[jacobian_index(3U, ind)] * joint_motion.qv[ind];
        result.linear.y += jacobian.j[jacobian_index(4U, ind)] * joint_motion.qv[ind];
        result.linear.z += jacobian.j[jacobian_index(5U, ind)] * joint_motion.qv[ind];
    }
    return result;
}

JointSpace jacobian_transpose_multiply(
    const Jacobian& jacobian, const MotionVector& cartesian_motion, size_t dofs)
{
    JointSpace result = {};
    if (dofs > MaxSize::dofs)
    {
        dofs = MaxSize::dofs;
    }
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        result.qv[ind] =
            jacobian.j[jacobian_index(0U, ind)] * cartesian_motion.angular.x +
            jacobian.j[jacobian_index(1U, ind)] * cartesian_motion.angular.y +
            jacobian.j[jacobian_index(2U, ind)] * cartesian_motion.angular.z +
            jacobian.j[jacobian_index(3U, ind)] * cartesian_motion.linear.x +
            jacobian.j[jacobian_index(4U, ind)] * cartesian_motion.linear.y +
            jacobian.j[jacobian_index(5U, ind)] * cartesian_motion.linear.z;
    }
    return result;
}

JointMatrix jacobian_transpose_multiply_jacobian(const Jacobian& jacobian, const MotionVector& cartesian_weights, size_t dofs)
{
    if (dofs > MaxSize::dofs)
    {
        dofs = MaxSize::dofs;
    }
    Jacobian scaled_jacobian = {};
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        scaled_jacobian.j[jacobian_index(0U, ind)] = jacobian.j[jacobian_index(0U, ind)] * cartesian_weights.angular.x;
        scaled_jacobian.j[jacobian_index(1U, ind)] = jacobian.j[jacobian_index(1U, ind)] * cartesian_weights.angular.y;
        scaled_jacobian.j[jacobian_index(2U, ind)] = jacobian.j[jacobian_index(2U, ind)] * cartesian_weights.angular.z;
        scaled_jacobian.j[jacobian_index(3U, ind)] = jacobian.j[jacobian_index(3U, ind)] * cartesian_weights.linear.x;
        scaled_jacobian.j[jacobian_index(4U, ind)] = jacobian.j[jacobian_index(4U, ind)] * cartesian_weights.linear.y;
        scaled_jacobian.j[jacobian_index(5U, ind)] = jacobian.j[jacobian_index(5U, ind)] * cartesian_weights.linear.z;
    }
    JointMatrix result = {};
    for (size_t col = 0U; col < dofs; ++col)
    {
        for (size_t row = 0U; row < dofs; ++row)
        {
            result.j[joint_matrix_index(row, col, dofs)] =
                jacobian.j[jacobian_index(0U, row)] * scaled_jacobian.j[jacobian_index(0U, col)] +
                jacobian.j[jacobian_index(1U, row)] * scaled_jacobian.j[jacobian_index(1U, col)] +
                jacobian.j[jacobian_index(2U, row)] * scaled_jacobian.j[jacobian_index(2U, col)] +
                jacobian.j[jacobian_index(3U, row)] * scaled_jacobian.j[jacobian_index(3U, col)] +
                jacobian.j[jacobian_index(4U, row)] * scaled_jacobian.j[jacobian_index(4U, col)] +
                jacobian.j[jacobian_index(5U, row)] * scaled_jacobian.j[jacobian_index(5U, col)];
        }
    }
    return result;
}

Jacobian jacobian_derivative(
    const Hessian& hessian, const JointSpace& joint_velocity, size_t dofs)
{
    Jacobian result = {};
    if (dofs > MaxSize::dofs)
    {
        dofs = MaxSize::dofs;
    }
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        const MotionVector result_col = jacobian_multiply(hessian.h[ind], joint_velocity);
        result.j[jacobian_index(0U, ind)] = result_col.angular.x;
        result.j[jacobian_index(1U, ind)] = result_col.angular.y;
        result.j[jacobian_index(2U, ind)] = result_col.angular.z;
        result.j[jacobian_index(3U, ind)] = result_col.linear.x;
        result.j[jacobian_index(4U, ind)] = result_col.linear.y;
        result.j[jacobian_index(5U, ind)] = result_col.linear.z;
    }
    return result;
}

} // namespace fsb
