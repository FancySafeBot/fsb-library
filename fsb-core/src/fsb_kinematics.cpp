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

void body_com_kinematics(
    const BodyTree& body_tree, const BodyCartesianPva& body_cartesian, BodyCartesianPva& com_cartesian)
{
    // propagate through tree
    const size_t num_bodies = body_tree.get_num_bodies();
    for (size_t body_index = 0U; body_index < num_bodies; ++body_index)
    {
        // get current child body and its parent joint from tree (error safely ignored)
        auto err = BodyTreeError::SUCCESS;
        const Vec3& com = body_tree.get_body(body_index, err).mass_props.com;
        const CartesianPva& body_pva = body_cartesian.body[body_index];
        com_cartesian.body[body_index].pose.rotation = body_pva.pose.rotation;
        com_cartesian.body[body_index].pose.translation = coord_transform_position(body_pva.pose, com);
        com_cartesian.body[body_index].velocity = motion_transform_velocity_position(body_pva.pose, body_pva.velocity, com);
        com_cartesian.body[body_index].acceleration = motion_transform_acceleration_position(body_pva.pose, body_pva.velocity, body_pva.acceleration, com);
    }
}

} // namespace fsb
