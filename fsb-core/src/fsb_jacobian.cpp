
#include <algorithm>
#include <array>
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
#include "fsb_linalg3.h"

namespace fsb
{

static Mat3Sym jacobian_multiply_jacobian_transpose_angular(const Jacobian& jacobian, size_t dofs)
{
    dofs = std::min(dofs, MaxSize::kDofs);
    Mat3Sym result = {};
    for (size_t col = 0U; col < dofs; ++col)
    {
        result.m00 += jacobian.j[jacobian_index(0U, col)] * jacobian.j[jacobian_index(0U, col)];
        result.m01 += jacobian.j[jacobian_index(0U, col)] * jacobian.j[jacobian_index(1U, col)];
        result.m02 += jacobian.j[jacobian_index(0U, col)] * jacobian.j[jacobian_index(2U, col)];
        result.m11 += jacobian.j[jacobian_index(1U, col)] * jacobian.j[jacobian_index(1U, col)];
        result.m22 += jacobian.j[jacobian_index(1U, col)] * jacobian.j[jacobian_index(2U, col)];
        result.m22 += jacobian.j[jacobian_index(2U, col)] * jacobian.j[jacobian_index(2U, col)];
    }
    return result;
}

static Mat3Sym jacobian_multiply_jacobian_transpose_linear(const Jacobian& jacobian, size_t dofs)
{
    dofs = std::min(dofs, MaxSize::kDofs);
    Mat3Sym result = {};
    for (size_t col = 0U; col < dofs; ++col)
    {
        result.m00 += jacobian.j[jacobian_index(3U, col)] * jacobian.j[jacobian_index(3U, col)];
        result.m01 += jacobian.j[jacobian_index(3U, col)] * jacobian.j[jacobian_index(4U, col)];
        result.m02 += jacobian.j[jacobian_index(3U, col)] * jacobian.j[jacobian_index(5U, col)];
        result.m11 += jacobian.j[jacobian_index(4U, col)] * jacobian.j[jacobian_index(4U, col)];
        result.m22 += jacobian.j[jacobian_index(4U, col)] * jacobian.j[jacobian_index(5U, col)];
        result.m22 += jacobian.j[jacobian_index(5U, col)] * jacobian.j[jacobian_index(5U, col)];
    }
    return result;
}

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
    const Real s_neg = joint.reversed ? -1.0 : 1.0;
    if (joint.type == JointType::REVOLUTE_X)
    {
        const size_t jac_col = joint.dof_index;

        // rotation component
        const Vec3 rot_x = quat_to_rotx(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col)] = s_neg * rot_x.x;
        jacobian.j[jacobian_index(1U, jac_col)] = s_neg * rot_x.y;
        jacobian.j[jacobian_index(2U, jac_col)] = s_neg * rot_x.z;

        // translation component
        const Vec3 pos_skew_x = vector_cross(
            vector_subtract(body_base_pose.translation, target_pose.translation), rot_x);
        jacobian.j[jacobian_index(3U, jac_col)] = s_neg * pos_skew_x.x;
        jacobian.j[jacobian_index(4U, jac_col)] = s_neg * pos_skew_x.y;
        jacobian.j[jacobian_index(5U, jac_col)] = s_neg * pos_skew_x.z;
    }
    else if (joint.type == JointType::REVOLUTE_Y)
    {
        const size_t jac_col = joint.dof_index;

        // rotation component
        const Vec3 rot_y = quat_to_roty(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col)] = s_neg * rot_y.x;
        jacobian.j[jacobian_index(1U, jac_col)] = s_neg * rot_y.y;
        jacobian.j[jacobian_index(2U, jac_col)] = s_neg * rot_y.z;

        // translation component
        const Vec3 pos_skew_y = vector_cross(
            vector_subtract(body_base_pose.translation, target_pose.translation), rot_y);
        jacobian.j[jacobian_index(3U, jac_col)] = s_neg * pos_skew_y.x;
        jacobian.j[jacobian_index(4U, jac_col)] = s_neg * pos_skew_y.y;
        jacobian.j[jacobian_index(5U, jac_col)] = s_neg * pos_skew_y.z;
    }
    else if (joint.type == JointType::REVOLUTE_Z)
    {
        const size_t jac_col = joint.dof_index;

        // rotation component
        const Vec3 rot_z = quat_to_rotz(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_col)] = s_neg * rot_z.x;
        jacobian.j[jacobian_index(1U, jac_col)] = s_neg * rot_z.y;
        jacobian.j[jacobian_index(2U, jac_col)] = s_neg * rot_z.z;

        // translation component
        const Vec3 pos_skew_z = vector_cross(
            vector_subtract(body_base_pose.translation, target_pose.translation), rot_z);
        jacobian.j[jacobian_index(3U, jac_col)] = s_neg * pos_skew_z.x;
        jacobian.j[jacobian_index(4U, jac_col)] = s_neg * pos_skew_z.y;
        jacobian.j[jacobian_index(5U, jac_col)] = s_neg * pos_skew_z.z;
    }
    else if (joint.type == JointType::PRISMATIC_X)
    {
        const size_t jac_col = joint.dof_index;

        // translation component
        const Vec3 rot_x = quat_to_rotx(body_base_pose.rotation);
        jacobian.j[jacobian_index(3U, jac_col)] = s_neg * rot_x.x;
        jacobian.j[jacobian_index(4U, jac_col)] = s_neg * rot_x.y;
        jacobian.j[jacobian_index(5U, jac_col)] = s_neg * rot_x.z;
    }
    else if (joint.type == JointType::PRISMATIC_Y)
    {
        const size_t jac_col = joint.dof_index;

        // translation component
        const Vec3 rot_y = quat_to_roty(body_base_pose.rotation);
        jacobian.j[jacobian_index(3U, jac_col)] = s_neg * rot_y.x;
        jacobian.j[jacobian_index(4U, jac_col)] = s_neg * rot_y.y;
        jacobian.j[jacobian_index(5U, jac_col)] = s_neg * rot_y.z;
    }
    else if (joint.type == JointType::PRISMATIC_Z)
    {
        const size_t jac_col = joint.dof_index;

        // translation component
        const Vec3 rot_z = quat_to_rotz(body_base_pose.rotation);
        jacobian.j[jacobian_index(3U, jac_col)] = s_neg * rot_z.x;
        jacobian.j[jacobian_index(4U, jac_col)] = s_neg * rot_z.y;
        jacobian.j[jacobian_index(5U, jac_col)] = s_neg * rot_z.z;
    }
    else if (joint.type == JointType::SPHERICAL)
    {
        const size_t jac_co_l0 = joint.dof_index;
        const size_t jac_co_l1 = joint.dof_index + 1U;
        const size_t jac_co_l2 = joint.dof_index + 2U;

        // jac: angular
        const Mat3 body_rot = quat_to_rot(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_co_l0)] = body_rot.m00;
        jacobian.j[jacobian_index(1U, jac_co_l0)] = body_rot.m10;
        jacobian.j[jacobian_index(2U, jac_co_l0)] = body_rot.m20;
        jacobian.j[jacobian_index(0U, jac_co_l1)] = body_rot.m01;
        jacobian.j[jacobian_index(1U, jac_co_l1)] = body_rot.m11;
        jacobian.j[jacobian_index(2U, jac_co_l1)] = body_rot.m21;
        jacobian.j[jacobian_index(0U, jac_co_l2)] = body_rot.m02;
        jacobian.j[jacobian_index(1U, jac_co_l2)] = body_rot.m12;
        jacobian.j[jacobian_index(2U, jac_co_l2)] = body_rot.m22;

        // jac: linear
        const Mat3 skew_rot = pos_skew_rot(
            vector_subtract(body_base_pose.translation, target_pose.translation), body_rot);
        jacobian.j[jacobian_index(3U, jac_co_l0)] = skew_rot.m00;
        jacobian.j[jacobian_index(4U, jac_co_l0)] = skew_rot.m10;
        jacobian.j[jacobian_index(5U, jac_co_l0)] = skew_rot.m20;
        jacobian.j[jacobian_index(3U, jac_co_l1)] = skew_rot.m01;
        jacobian.j[jacobian_index(4U, jac_co_l1)] = skew_rot.m11;
        jacobian.j[jacobian_index(5U, jac_co_l1)] = skew_rot.m21;
        jacobian.j[jacobian_index(3U, jac_co_l2)] = skew_rot.m02;
        jacobian.j[jacobian_index(4U, jac_co_l2)] = skew_rot.m12;
        jacobian.j[jacobian_index(5U, jac_co_l2)] = skew_rot.m22;
    }
    else if (joint.type == JointType::CARTESIAN)
    {
        const size_t jac_co_l0 = joint.dof_index;
        const size_t jac_co_l1 = joint.dof_index + 1U;
        const size_t jac_co_l2 = joint.dof_index + 2U;
        const size_t jac_co_l3 = joint.dof_index + 3U;
        const size_t jac_co_l4 = joint.dof_index + 4U;
        const size_t jac_co_l5 = joint.dof_index + 5U;

        // jac sub-matrix: linear
        const Mat3 body_rot = quat_to_rot(body_base_pose.rotation);
        jacobian.j[jacobian_index(0U, jac_co_l0)] = body_rot.m00;
        jacobian.j[jacobian_index(1U, jac_co_l0)] = body_rot.m10;
        jacobian.j[jacobian_index(2U, jac_co_l0)] = body_rot.m20;
        jacobian.j[jacobian_index(0U, jac_co_l1)] = body_rot.m01;
        jacobian.j[jacobian_index(1U, jac_co_l1)] = body_rot.m11;
        jacobian.j[jacobian_index(2U, jac_co_l1)] = body_rot.m21;
        jacobian.j[jacobian_index(0U, jac_co_l2)] = body_rot.m02;
        jacobian.j[jacobian_index(1U, jac_co_l2)] = body_rot.m12;
        jacobian.j[jacobian_index(2U, jac_co_l2)] = body_rot.m22;

        // jac sub-matrix: angular
        const Mat3 skew_rot = pos_skew_rot(
            vector_subtract(body_base_pose.translation, target_pose.translation), body_rot);
        jacobian.j[jacobian_index(3U, jac_co_l0)] = skew_rot.m00;
        jacobian.j[jacobian_index(4U, jac_co_l0)] = skew_rot.m10;
        jacobian.j[jacobian_index(5U, jac_co_l0)] = skew_rot.m20;
        jacobian.j[jacobian_index(3U, jac_co_l1)] = skew_rot.m01;
        jacobian.j[jacobian_index(4U, jac_co_l1)] = skew_rot.m11;
        jacobian.j[jacobian_index(5U, jac_co_l1)] = skew_rot.m21;
        jacobian.j[jacobian_index(3U, jac_co_l2)] = skew_rot.m02;
        jacobian.j[jacobian_index(4U, jac_co_l2)] = skew_rot.m12;
        jacobian.j[jacobian_index(5U, jac_co_l2)] = skew_rot.m22;

        jacobian.j[jacobian_index(3U, jac_co_l3)] = body_rot.m00;
        jacobian.j[jacobian_index(4U, jac_co_l3)] = body_rot.m10;
        jacobian.j[jacobian_index(5U, jac_co_l3)] = body_rot.m20;
        jacobian.j[jacobian_index(3U, jac_co_l4)] = body_rot.m01;
        jacobian.j[jacobian_index(4U, jac_co_l4)] = body_rot.m11;
        jacobian.j[jacobian_index(5U, jac_co_l4)] = body_rot.m21;
        jacobian.j[jacobian_index(3U, jac_co_l5)] = body_rot.m02;
        jacobian.j[jacobian_index(4U, jac_co_l5)] = body_rot.m12;
        jacobian.j[jacobian_index(5U, jac_co_l5)] = body_rot.m22;
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
            const Transform  body_base_pose
                = coord_transform(parent_pose, joint.parent_joint_transform);
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

static bool is_revolute(const JointType jt)
{
    return (jt == JointType::REVOLUTE_X) || (jt == JointType::REVOLUTE_Y)
           || (jt == JointType::REVOLUTE_Z);
}

static bool is_prismatic(const JointType jt)
{
    return (jt == JointType::PRISMATIC_X) || (jt == JointType::PRISMATIC_Y)
           || (jt == JointType::PRISMATIC_Z);
}

static Jacobian jacobian_derivative_all_revolute_prismatic(
    const std::array<JointType, MaxSize::kDofs>& dof_joint_type, const Jacobian& jacobian,
    const JointSpace& joint_velocity, const size_t dofs)
{
    Jacobian result = {};
    // Exact derivative of jacobian (for revolute/prismatic chains)
    for (size_t k = 0; k < dofs; ++k)
    {
        // jwk: angular part of column k
        const Vec3 jwk
            = {jacobian.j[jacobian_index(0U, k)],
               jacobian.j[jacobian_index(1U, k)],
               jacobian.j[jacobian_index(2U, k)]};
        // jvk: linear part of column k
        const Vec3 jvk
            = {jacobian.j[jacobian_index(3U, k)],
               jacobian.j[jacobian_index(4U, k)],
               jacobian.j[jacobian_index(5U, k)]};

        for (size_t i = 0; i <= k; ++i)
        {
            // Hessian slice i, column k
            if (is_revolute(dof_joint_type[i]))
            {
                // jwi: angular part of Jacobian column i
                const Vec3 jwi
                    = {jacobian.j[jacobian_index(0U, i)],
                       jacobian.j[jacobian_index(1U, i)],
                       jacobian.j[jacobian_index(2U, i)]};
                // linear part of of Hessian slice i, column k
                const Vec3 jvk_dqi = vector_cross(jwi, jvk);
                result.j[jacobian_index(3U, k)] += jvk_dqi.x * joint_velocity[i];
                result.j[jacobian_index(4U, k)] += jvk_dqi.y * joint_velocity[i];
                result.j[jacobian_index(5U, k)] += jvk_dqi.z * joint_velocity[i];

                // angular part of Hessian slice i, column k
                if (is_revolute(dof_joint_type[k]) && (i != k))
                {
                    const Vec3 jwk_dqi = vector_cross(jwi, jwk);
                    result.j[jacobian_index(0U, k)] += jwk_dqi.x * joint_velocity[i];
                    result.j[jacobian_index(1U, k)] += jwk_dqi.y * joint_velocity[i];
                    result.j[jacobian_index(2U, k)] += jwk_dqi.z * joint_velocity[i];
                }

                // linear part of of Hessian slice k, column i (symmetric with slice i, column k)
                // jvi_dqk = jvk_dqi, off-diagonal only
                if (i != k)
                {
                    result.j[jacobian_index(3U, i)] += jvk_dqi.x * joint_velocity[k];
                    result.j[jacobian_index(4U, i)] += jvk_dqi.y * joint_velocity[k];
                    result.j[jacobian_index(5U, i)] += jvk_dqi.z * joint_velocity[k];
                }
            }
        }
    }
    return result;
}

JacobianError jacobian_derivative(
    const size_t body_index, const BodyTree& body_tree, const Jacobian& jacobian,
    const JointSpace& joint_velocity, Jacobian& jacobian_deriv)
{
    auto body_err = BodyTreeError::SUCCESS;
    const size_t  dofs = body_tree.get_body_dofs(body_index, body_err);

    // propagate through parent bodies
    bool all_revolute_or_prismatic = true;
    std::array<JointType, MaxSize::kDofs> dof_joint_type = {JointType::FIXED};
    size_t temp_body_index = body_index;
    while ((temp_body_index > 0U) && all_revolute_or_prismatic)
    {
        BodyTreeError err = BodyTreeError::SUCCESS;
        const Joint& joint = body_tree.get_joint(temp_body_index, err);
        dof_joint_type[joint.dof_index] = joint.type;
        if (!is_revolute(joint.type) && !is_prismatic(joint.type) && joint.type != JointType::FIXED)
        {
            all_revolute_or_prismatic = false;
        }
        // next parent
        temp_body_index = joint.parent_body_index;
    }

    auto err = JacobianError::SUCCESS;
    if (all_revolute_or_prismatic)
    {
        jacobian_deriv = jacobian_derivative_all_revolute_prismatic(
            dof_joint_type, jacobian, joint_velocity, dofs);
    }
    else
    {
        err = JacobianError::NOT_REVOLUTE_OR_PRISMATIC;
    }
    return err;
}

static Hessian calculate_hessian_all_revolute_prismatic(
    const std::array<JointType, MaxSize::kDofs>& dof_joint_type, const Jacobian& jacobian,
    const size_t dofs)
{
    Hessian result = {};
    // Exact derivative of jacobian (for revolute/prismatic chains)
    for (size_t k = 0; k < dofs; ++k)
    {
        // jwk: angular part of column k
        const Vec3 jwk
            = {jacobian.j[jacobian_index(0U, k)],
               jacobian.j[jacobian_index(1U, k)],
               jacobian.j[jacobian_index(2U, k)]};
        // jvk: linear part of column k
        const Vec3 jvk
            = {jacobian.j[jacobian_index(3U, k)],
               jacobian.j[jacobian_index(4U, k)],
               jacobian.j[jacobian_index(5U, k)]};

        for (size_t i = 0; i <= k; ++i)
        {
            // Hessian slice i, column k
            if (is_revolute(dof_joint_type[i]))
            {
                // jwi: angular part of Jacobian column i
                const Vec3 jwi
                    = {jacobian.j[jacobian_index(0U, i)],
                       jacobian.j[jacobian_index(1U, i)],
                       jacobian.j[jacobian_index(2U, i)]};
                // linear part of of Hessian slice i, column k
                const Vec3 jvk_dqi = vector_cross(jwi, jvk);
                result.h[i].j[jacobian_index(3U, k)] = jvk_dqi.x;
                result.h[i].j[jacobian_index(4U, k)] = jvk_dqi.y;
                result.h[i].j[jacobian_index(5U, k)] = jvk_dqi.z;

                // angular part of Hessian slice i, column k
                if (is_revolute(dof_joint_type[k]) && (i != k))
                {
                    const Vec3 jwk_dqi = vector_cross(jwi, jwk);
                    result.h[i].j[jacobian_index(0U, k)] = jwk_dqi.x;
                    result.h[i].j[jacobian_index(1U, k)] = jwk_dqi.y;
                    result.h[i].j[jacobian_index(2U, k)] = jwk_dqi.z;
                }

                // linear part of of Hessian slice k, column i (symmetric with slice i, column k)
                // jvi_dqk = jvk_dqi, off-diagonal only
                if (i != k)
                {
                    result.h[k].j[jacobian_index(3U, i)] = jvk_dqi.x;
                    result.h[k].j[jacobian_index(4U, i)] = jvk_dqi.y;
                    result.h[k].j[jacobian_index(5U, i)] = jvk_dqi.z;
                }
            }
        }
    }
    return result;
}

JacobianError calculate_hessian(
    const size_t body_index, const BodyTree& body_tree, const Jacobian& jacobian, Hessian& hessian)
{
    JacobianError err = JacobianError::SUCCESS;
    // initialize
    auto body_err = BodyTreeError::SUCCESS;
    const size_t  dofs = body_tree.get_body_dofs(body_index, body_err);

    // propagate through parent bodies
    bool all_revolute_or_prismatic = true;
    std::array<JointType, MaxSize::kDofs> dof_joint_type = {JointType::FIXED};
    size_t                               temp_body_index = body_index;
    while ((temp_body_index > 0U) && all_revolute_or_prismatic)
    {
        body_err = BodyTreeError::SUCCESS;
        const Joint& joint = body_tree.get_joint(temp_body_index, body_err);
        dof_joint_type[joint.dof_index] = joint.type;
        if (!is_revolute(joint.type) && !is_prismatic(joint.type) && joint.type != JointType::FIXED)
        {
            all_revolute_or_prismatic = false;
        }
        // next parent
        temp_body_index = joint.parent_body_index;
    }

    if (all_revolute_or_prismatic)
    {
        hessian = calculate_hessian_all_revolute_prismatic(dof_joint_type, jacobian, dofs);
        err = JacobianError::SUCCESS;
    }
    else
    {
        err = JacobianError::NOT_REVOLUTE_OR_PRISMATIC;
    }
    return err;
}

Jacobian hessian_multiply(
    const Hessian& hessian, const JointSpace& joint_motion, size_t dofs)
{
    Jacobian result = {};
    dofs = std::min(dofs, MaxSize::kDofs);
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        const MotionVector result_col = jacobian_multiply(hessian.h[ind], joint_motion);
        result.j[jacobian_index(0U, ind)] = result_col.angular.x;
        result.j[jacobian_index(1U, ind)] = result_col.angular.y;
        result.j[jacobian_index(2U, ind)] = result_col.angular.z;
        result.j[jacobian_index(3U, ind)] = result_col.linear.x;
        result.j[jacobian_index(4U, ind)] = result_col.linear.y;
        result.j[jacobian_index(5U, ind)] = result_col.linear.z;
    }
    return result;
}

MotionVector jacobian_multiply(
    const Jacobian& jacobian, const JointSpace& joint_motion, size_t dofs)
{
    MotionVector result = {};
    dofs = std::min(dofs, MaxSize::kDofs);
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        result.angular.x += jacobian.j[jacobian_index(0U, ind)] * joint_motion[ind];
        result.angular.y += jacobian.j[jacobian_index(1U, ind)] * joint_motion[ind];
        result.angular.z += jacobian.j[jacobian_index(2U, ind)] * joint_motion[ind];
        result.linear.x += jacobian.j[jacobian_index(3U, ind)] * joint_motion[ind];
        result.linear.y += jacobian.j[jacobian_index(4U, ind)] * joint_motion[ind];
        result.linear.z += jacobian.j[jacobian_index(5U, ind)] * joint_motion[ind];
    }
    return result;
}

JointSpace jacobian_transpose_multiply(
    const Jacobian& jacobian, const MotionVector& cartesian_motion, size_t dofs)
{
    JointSpace result = {};
    dofs = std::min(dofs, MaxSize::kDofs);
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        result[ind] =
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
    dofs = std::min(dofs, MaxSize::kDofs);
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

Jacobian jacobian_derivative_from_hessian(const Hessian& hessian, const JointSpace& joint_velocity, size_t dofs)
{
    Jacobian result = {};
    dofs = std::min(dofs, MaxSize::kDofs);
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

JacobianMetrics calculate_jacobian_metrics(const Jacobian& jacobian, const size_t dofs)
{
    JacobianMetrics result = {};

    // A = J * J.transpose()
    // [U, S, VT] = svd(A)
    const Mat3Sym lin_mat = jacobian_multiply_jacobian_transpose_linear(jacobian, dofs);
    Vec3 lin_eig_vals = {};
    Vec3 lin_eig_vec0 = {};
    Vec3 lin_eig_vec1 = {};
    Vec3 lin_eig_vec2 = {};
    bool          lin_eig_ok = mat3_posdef_symmetric_eigenvalues(lin_mat, lin_eig_vals);
    if (lin_eig_ok)
    {
        lin_eig_ok = mat3_posdef_symmetric_eigenvectors(
            lin_mat, lin_eig_vals, lin_eig_vec0, lin_eig_vec1, lin_eig_vec2);
    }

    // A = J * J.transpose()
    // [U, S, VT] = svd(A)
    const Mat3Sym ang_mat = jacobian_multiply_jacobian_transpose_angular(jacobian, dofs);
    Vec3 ang_eig_vals = {};
    Vec3 ang_eig_vec0 = {};
    Vec3 ang_eig_vec1 = {};
    Vec3 ang_eig_vec2 = {};
    bool          ang_eig_ok = mat3_posdef_symmetric_eigenvalues(ang_mat, ang_eig_vals);
    if (ang_eig_ok)
    {
        ang_eig_ok = mat3_posdef_symmetric_eigenvectors(
            ang_mat, ang_eig_vals, ang_eig_vec0, ang_eig_vec1, ang_eig_vec2);
    }

    result.angular.is_singular = !ang_eig_ok;
    if (ang_eig_ok)
    {
        result.angular.condition_number = ang_eig_vals.z / ang_eig_vals.x;
        result.angular.ellipsoid.eig_values = ang_eig_vals;
        result.angular.ellipsoid.eig_vectors = {
            ang_eig_vec0.x, ang_eig_vec0.y, ang_eig_vec0.z,
            ang_eig_vec1.x, ang_eig_vec1.y, ang_eig_vec1.z,
            ang_eig_vec2.x, ang_eig_vec2.y, ang_eig_vec2.z};
    }

    result.linear.is_singular = !lin_eig_ok;
    if (lin_eig_ok)
    {
        result.linear.condition_number = lin_eig_vals.z / lin_eig_vals.x;
        result.linear.ellipsoid.eig_values = lin_eig_vals;
        result.linear.ellipsoid.eig_vectors = {
            lin_eig_vec0.x, lin_eig_vec0.y, lin_eig_vec0.z,
            lin_eig_vec1.x, lin_eig_vec1.y, lin_eig_vec1.z,
            lin_eig_vec2.x, lin_eig_vec2.y, lin_eig_vec2.z};
    }

    return result;
}

} // namespace fsb
