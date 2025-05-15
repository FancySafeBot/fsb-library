#include <cstddef>
#include "fsb_dynamics.h"
#include "fsb_motion.h"
#include "fsb_body.h"
#include "fsb_joint.h"
#include "fsb_quaternion.h"
#include "fsb_rotation.h"
#include "fsb_types.h"
#include "fsb_body_tree.h"

namespace fsb
{

static BodyForce compute_body_com_force(const BodyTree& body_tree, const BodyCartesianPva& cartesian_motion, const BodyForce& external_force)
{
    BodyForce result = {};
    const Vec3 gravity = body_tree.get_gravity();
    for (size_t index = 0; index < body_tree.get_num_bodies(); ++index)
    {
        BodyTreeError err = {};
        // Body mass properties
        const Body& body = body_tree.get_body(index, err);
        const Vec3& com = body.mass_props.com;
        const real_t mass = body.mass_props.mass;
        const Inertia& inertia = body.mass_props.inertia;
        // Body motion in body-fixed coordinates
        const Transform& pose = cartesian_motion.body[index].pose;
        const Mat3 rot = quat_to_rot(pose.rotation);
        const Vec3 acc_linear = rotate_mat3_transpose(rot, cartesian_motion.body[index].acceleration.linear);
        const Vec3 acc_angular = rotate_mat3_transpose(rot, cartesian_motion.body[index].acceleration.angular);
        const Vec3 vel_angular = rotate_mat3_transpose(rot, cartesian_motion.body[index].velocity.angular);
        // Compute linear force
        const Vec3 acc_com_normal = vector_cross(vel_angular, vector_cross(vel_angular, com));
        const Vec3 acc_com = vector_add(vector_cross(acc_angular, com), acc_com_normal);
        const Vec3 acc_com_gravity = vector_subtract(acc_com, rotate_mat3_transpose(rot, gravity));
        const Vec3 force_dyn = vector_scale(mass, vector_add(acc_linear, acc_com_gravity));
        // Compute torque
        const Vec3 torque_dyn = vector_add(inertia_multiply_vector(inertia, acc_angular), inertia_cross_multiply_vector(inertia, vel_angular));
        const Vec3 torque_from_force_com = vector_cross(com, force_dyn);
        // External forces
        const Vec3 torque_ext = rotate_mat3_transpose(rot, external_force.body[index].torque);
        const Vec3 force_ext = rotate_mat3_transpose(rot, external_force.body[index].force);
        // Result
        result.body[index].force = vector_subtract(force_dyn, force_ext);
        result.body[index].torque = vector_add(vector_subtract(torque_dyn, torque_ext), torque_from_force_com);
    }
    return result;
}

static void joint_torque_from_force(const Joint& joint, const ForceVector& joint_force, JointSpace& joint_torque)
{
    if (joint.type == JointType::REVOLUTE_X)
    {
        joint_torque.qv[joint.dof_index] = joint_force.torque.x;
    }
    if (joint.type == JointType::REVOLUTE_Y)
    {
        joint_torque.qv[joint.dof_index] = joint_force.torque.y;
    }
    if (joint.type == JointType::REVOLUTE_Z)
    {
        joint_torque.qv[joint.dof_index] = joint_force.torque.z;
    }
    else if (joint.type == JointType::PRISMATIC_X)
    {
        joint_torque.qv[joint.dof_index] = joint_force.force.x;
    }
    else if (joint.type == JointType::PRISMATIC_Y)
    {
        joint_torque.qv[joint.dof_index] = joint_force.force.y;
    }
    else if (joint.type == JointType::PRISMATIC_Z)
    {
        joint_torque.qv[joint.dof_index] = joint_force.force.z;
    }
    else if (joint.type == JointType::SPHERICAL)
    {
        joint_torque.qv[joint.dof_index] = joint_force.torque.x;
        joint_torque.qv[joint.dof_index + 1U] = joint_force.torque.y;
        joint_torque.qv[joint.dof_index + 2U] = joint_force.torque.z;
    }
    else if (joint.type == JointType::CARTESIAN)
    {
        joint_torque.qv[joint.dof_index] = joint_force.torque.x;
        joint_torque.qv[joint.dof_index + 1U] = joint_force.torque.y;
        joint_torque.qv[joint.dof_index + 2U] = joint_force.torque.z;

        joint_torque.qv[joint.dof_index + 3U] = joint_force.force.x;
        joint_torque.qv[joint.dof_index + 4U] = joint_force.force.y;
        joint_torque.qv[joint.dof_index + 5U] = joint_force.force.z;
    }
    else
    {
        // joint.type is JointType::FIXED
        // result is identity transform and zero velocity
    }
}

static JointSpace compute_body_joint_forces(const BodyTree& body_tree, const BodyCartesianPva& cartesian_motion, BodyForce& body_force)
{
    JointSpace result = {};
    for (size_t index = body_tree.get_num_bodies() - 1U; index > 0U; --index)
    {
        BodyTreeError err = {};
        // Get parent body and joint
        const Body& body = body_tree.get_body(index, err);
        const Joint& joint = body_tree.get_joint(body.joint_index, err);
        const size_t parent_body_index = joint.parent_body_index;
        // get joint torque
        joint_torque_from_force(joint, body_force.body[index], result);
        // Parent child transform
        const Transform& pose = cartesian_motion.body[index].pose;
        const Transform& parent_pose = cartesian_motion.body[parent_body_index].pose;
        const Transform parent_child_transform = coord_transform_inverse(parent_pose, pose);
        // body joint force
        const Vec3 force_child = quat_rotate_vector(parent_child_transform.rotation, body_force.body[index].force);
        body_force.body[parent_body_index].force = vector_add(body_force.body[parent_body_index].force, force_child);
        // body joint torque
        const Vec3 torque_child = quat_rotate_vector(parent_child_transform.rotation, body_force.body[index].torque);
        const Vec3 torque_from_force = vector_cross(parent_child_transform.translation, force_child);
        body_force.body[parent_body_index].torque = vector_add(body_force.body[parent_body_index].torque,
            vector_add(torque_child, torque_from_force));
    }
    return result;
}

JointSpace inverse_dynamics(
    const BodyTree& body_tree, const BodyCartesianPva& cartesian_motion, const BodyForce& external_force,
    BodyForce& body_force)
{
    // Compute body forces
    body_force = compute_body_com_force(body_tree, cartesian_motion, external_force);
    // Compute joint torques
    return compute_body_joint_forces(body_tree, cartesian_motion, body_force);
}

}
