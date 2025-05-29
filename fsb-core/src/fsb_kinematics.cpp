#include <cstddef>

#include "fsb_types.h"
#include "fsb_kinematics.h"
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_quaternion.h"
#include "fsb_motion.h"
#include "fsb_joint.h"

namespace fsb
{

static void compute_child_kinematics(
    const CartesianPva& body_pva, const Joint& joint, const JointPva& joint_pva, const ForwardKinematicsOption opt,
    CartesianPva& child_body_pva)
{
    const Transform tr_parent_child = joint_parent_child_transform(joint, joint_pva.position);
    child_body_pva.pose = coord_transform(body_pva.pose, tr_parent_child);

    if ((opt == ForwardKinematicsOption::POSE_VELOCITY) || (opt == ForwardKinematicsOption::POSE_VELOCITY_ACCELERATION))
    {
        // motion velocity transform
        const MotionVector vel_parent_child = joint_parent_child_velocity(joint, joint_pva.velocity);
        child_body_pva.velocity
            = motion_transform_velocity(body_pva.pose, body_pva.velocity, tr_parent_child, vel_parent_child);

        if (opt == ForwardKinematicsOption::POSE_VELOCITY_ACCELERATION)
        {
            // motion acceleration transform
            child_body_pva.acceleration = motion_transform_acceleration(
                body_pva.pose, body_pva.velocity, body_pva.acceleration, tr_parent_child, vel_parent_child,
                joint_parent_child_acceleration(joint, joint_pva));
        }
    }
}

void forward_kinematics(
    const BodyTree& body_tree, const JointPva& joint_pva, const CartesianPva& base_pva,
    const ForwardKinematicsOption opt, BodyCartesianPva& body_cartesian)
{
    // set base pva, ensure base quaternion is normalized
    constexpr size_t base_index = 0U;
    body_cartesian.body[base_index] = base_pva;
    quat_normalize(body_cartesian.body[base_index].pose.rotation);
    // propagate through tree
    const size_t num_bodies = body_tree.get_num_bodies();
    for (size_t body_index = 1U; body_index < num_bodies; ++body_index)
    {
        // get current child body and its parent joint from tree
        auto err = BodyTreeError::SUCCESS;
        const Body& body = body_tree.get_body(body_index, err);
        // err does not need to be checked since body_tree.get_num_bodies() ensure body and joint indexes are valid
        const Joint& joint = body_tree.get_joint(body.joint_index, err);
        // get parent body pva
        const CartesianPva& body_pva = body_cartesian.body[joint.parent_body_index];
        // compute child pva
        CartesianPva& child_body_pva = body_cartesian.body[joint.child_body_index];
        compute_child_kinematics(body_pva, joint, joint_pva, opt, child_body_pva);
    }
}

JointSpacePosition joint_add_offset(
    const BodyTree& body_tree, const JointSpacePosition& joint_position,
    const JointSpace& joint_offset)
{
    JointSpacePosition result = joint_position;
    const size_t num_bodies = body_tree.get_num_bodies();

    for (size_t body_index = 1U; body_index < num_bodies; ++body_index)
    {
        auto err = BodyTreeError::SUCCESS;
        const Body& body = body_tree.get_body(body_index, err);
        const Joint& joint = body_tree.get_joint(body.joint_index, err);
        switch (joint.type)
        {
            case JointType::SPHERICAL:
            {
                // Compose quaternions: q_new = q_offset * q_current
                const Quaternion q_current = {
                    result.q[joint.coord_index + 0U],
                    result.q[joint.coord_index + 1U],
                    result.q[joint.coord_index + 2U],
                    result.q[joint.coord_index + 3U]
                };
                const Vec3 q_offset = {
                    joint_offset.qv[joint.dof_index + 0U],
                    joint_offset.qv[joint.dof_index + 1U],
                    joint_offset.qv[joint.dof_index + 2U]
                };
                const Quaternion q_new = quat_boxplus(q_current, q_offset);
                result.q[joint.coord_index + 0U] = q_new.qw;
                result.q[joint.coord_index + 1U] = q_new.qx;
                result.q[joint.coord_index + 2U] = q_new.qy;
                result.q[joint.coord_index + 3U] = q_new.qz;
                break;
            }
            case JointType::CARTESIAN:
            {
                // Compose quaternions: q_new = q_offset * q_current
                const Quaternion q_current = {
                    result.q[joint.coord_index + 0U],
                    result.q[joint.coord_index + 1U],
                    result.q[joint.coord_index + 2U],
                    result.q[joint.coord_index + 3U]
                };
                const Vec3 q_offset = {
                    joint_offset.qv[joint.dof_index + 0U],
                    joint_offset.qv[joint.dof_index + 1U],
                    joint_offset.qv[joint.dof_index + 2U]
                };
                const Quaternion q_new = quat_boxplus(q_current, q_offset);
                result.q[joint.coord_index + 0U] = q_new.qw;
                result.q[joint.coord_index + 1U] = q_new.qx;
                result.q[joint.coord_index + 2U] = q_new.qy;
                result.q[joint.coord_index + 3U] = q_new.qz;
                // Add translation part
                for (size_t ind = 0; ind < 3; ++ind)
                {
                    result.q[joint.coord_index + 4U + ind] += joint_offset.qv[joint.dof_index + 4U + ind];
                }
                break;
            }
            case JointType::REVOLUTE_X:
            case JointType::REVOLUTE_Y:
            case JointType::REVOLUTE_Z:
            case JointType::PRISMATIC_X:
            case JointType::PRISMATIC_Y:
            case JointType::PRISMATIC_Z:
            {
                // single dof offset
                result.q[joint.coord_index] += joint_offset.qv[joint.dof_index];
                break;
            }
            case JointType::PLANAR:
            {
                // three dof offset
                result.q[joint.coord_index] += joint_offset.qv[joint.dof_index];
                result.q[joint.coord_index + 1U] += joint_offset.qv[joint.dof_index + 1U];
                result.q[joint.coord_index + 2U] += joint_offset.qv[joint.dof_index + 2U];
                break;
            }
            case JointType::FIXED:
            default:
                // FIXED or unknown: do nothing
                break;
        }
    }
    return result;
}

JointSpace joint_difference(
    const BodyTree& body_tree, const JointSpacePosition& joint_position_a,
    const JointSpacePosition& joint_position_b)
{
    JointSpace result = {};
    const size_t num_bodies = body_tree.get_num_bodies();
    for (size_t body_index = 1U; body_index < num_bodies; ++body_index)
    {
        auto err = BodyTreeError::SUCCESS;
        const Body& body = body_tree.get_body(body_index, err);
        const Joint& joint = body_tree.get_joint(body.joint_index, err);
        switch (joint.type)
        {
            case JointType::SPHERICAL:
            {
                // SO(3) difference: log(q_a * inv(q_b))
                const Quaternion q_a = {
                    joint_position_a.q[joint.coord_index + 0U],
                    joint_position_a.q[joint.coord_index + 1U],
                    joint_position_a.q[joint.coord_index + 2U],
                    joint_position_a.q[joint.coord_index + 3U]
                };
                const Quaternion q_b = {
                    joint_position_b.q[joint.coord_index + 0U],
                    joint_position_b.q[joint.coord_index + 1U],
                    joint_position_b.q[joint.coord_index + 2U],
                    joint_position_b.q[joint.coord_index + 3U]
                };
                const Vec3 diff = quat_boxminus(q_a, q_b);
                result.qv[joint.dof_index + 0U] = diff.x;
                result.qv[joint.dof_index + 1U] = diff.y;
                result.qv[joint.dof_index + 2U] = diff.z;
                break;
            }
            case JointType::CARTESIAN:
            {
                // SO(3) difference for rotation
                const Quaternion q_a = {
                    joint_position_a.q[joint.coord_index + 0U],
                    joint_position_a.q[joint.coord_index + 1U],
                    joint_position_a.q[joint.coord_index + 2U],
                    joint_position_a.q[joint.coord_index + 3U]
                };
                const Quaternion q_b = {
                    joint_position_b.q[joint.coord_index + 0U],
                    joint_position_b.q[joint.coord_index + 1U],
                    joint_position_b.q[joint.coord_index + 2U],
                    joint_position_b.q[joint.coord_index + 3U]
                };
                const Vec3 diff_rot = quat_boxminus(q_a, q_b);
                result.qv[joint.dof_index + 0U] = diff_rot.x;
                result.qv[joint.dof_index + 1U] = diff_rot.y;
                result.qv[joint.dof_index + 2U] = diff_rot.z;
                // translation difference
                for (size_t ind = 0; ind < 3; ++ind)
                {
                    result.qv[joint.dof_index + 3U + ind] =
                        joint_position_a.q[joint.coord_index + 4U + ind] -
                        joint_position_b.q[joint.coord_index + 4U + ind];
                }
                break;
            }
            case JointType::REVOLUTE_X:
            case JointType::REVOLUTE_Y:
            case JointType::REVOLUTE_Z:
            case JointType::PRISMATIC_X:
            case JointType::PRISMATIC_Y:
            case JointType::PRISMATIC_Z:
            {
                result.qv[joint.dof_index] =
                    joint_position_a.q[joint.coord_index] - joint_position_b.q[joint.coord_index];
                break;
            }
            case JointType::PLANAR:
            {
                // three dof offset
                result.qv[joint.dof_index] = joint_position_a.q[joint.coord_index] - joint_position_b.q[joint.coord_index];
                result.qv[joint.dof_index + 1U] = joint_position_a.q[joint.coord_index + 1U] - joint_position_b.q[joint.coord_index + 1U];
                result.qv[joint.dof_index + 2U] = joint_position_a.q[joint.coord_index + 2U] - joint_position_b.q[joint.coord_index + 2U];
                break;
            }
            case JointType::FIXED:
            default:
                // FIXED or unknown: do nothing
                break;
        }
    }
    return result;
}

} // namespace fsb
