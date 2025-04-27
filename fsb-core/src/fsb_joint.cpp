#include "fsb_joint.h"
#include "fsb_motion.h"
#include "fsb_quaternion.h"
#include "fsb_types.h"

namespace fsb
{

Transform joint_parent_child_transform(const Joint& joint, const JointSpacePosition& position)
{
    // alias constant transform to start of base
    const Transform& tr_pj = joint.parent_joint_transform;

    // initialize result
    Transform result = transform_identity();

    // determine pose from joint position
    if (joint.type == JointType::FIXED)
    {
        result = tr_pj;
    }
    else if (joint.type == JointType::REVOLUTE_X)
    {
        result = {
            quat_multiply(tr_pj.rotation, quat_rx(position.q[joint.coord_index])),
            tr_pj.translation
        };
    }
    else if (joint.type == JointType::REVOLUTE_Y)
    {
        result = {
            quat_multiply(tr_pj.rotation, quat_ry(position.q[joint.coord_index])),
            tr_pj.translation
        };
    }
    else if (joint.type == JointType::REVOLUTE_Z)
    {
        result = {
            quat_multiply(tr_pj.rotation, quat_rz(position.q[joint.coord_index])),
            tr_pj.translation
        };
    }
    else if (joint.type == JointType::PRISMATIC_X)
    {
        const Vec3 pos = quat_rotate_vector(tr_pj.rotation, {position.q[joint.coord_index], 0.0, 0.0});
        result = {tr_pj.rotation, vector_add(tr_pj.translation, pos)};
    }
    else if (joint.type == JointType::PRISMATIC_Y)
    {
        const Vec3 pos = quat_rotate_vector(tr_pj.rotation, {0.0, position.q[joint.coord_index], 0.0});
        result = {tr_pj.rotation, vector_add(tr_pj.translation, pos)};
    }
    else if (joint.type == JointType::PRISMATIC_Z)
    {
        const Vec3 pos = quat_rotate_vector(tr_pj.rotation, {0.0, 0.0, position.q[joint.coord_index]});
        result = {tr_pj.rotation, vector_add(tr_pj.translation, pos)};
    }
    else if (joint.type == JointType::SPHERICAL)
    {
        const Quaternion quat_joint
            = {position.q[joint.coord_index], position.q[joint.coord_index + 1U], position.q[joint.coord_index + 2U],
               position.q[joint.coord_index + 3U]};
        result = {quat_multiply(tr_pj.rotation, quat_joint), tr_pj.translation};
    }
    else if (joint.type == JointType::CARTESIAN)
    {
        const Transform tr_joint = {
            {position.q[joint.coord_index], position.q[joint.coord_index + 1U], position.q[joint.coord_index + 2U],
             position.q[joint.coord_index + 3U]},
            {position.q[joint.coord_index + 4U], position.q[joint.coord_index + 5U],
             position.q[joint.coord_index + 6U]}
        };
        result = coord_transform(tr_pj, tr_joint);
    }
    else
    {
        // result is identity transform
    }

    return result;
}

MotionVector joint_parent_child_velocity(const Joint& joint, const JointSpace& velocity)
{
    // alias constant transform to start of base
    const Transform& tr_pj = joint.parent_joint_transform;

    // initialize result
    MotionVector vel_result = {};

    // determine velocity from joint motion
    if (joint.type == JointType::REVOLUTE_X)
    {
        vel_result.angular = quat_rotate_vector(tr_pj.rotation, {velocity.qv[joint.dof_index], 0.0, 0.0});
    }
    if (joint.type == JointType::REVOLUTE_Y)
    {
        vel_result.angular = quat_rotate_vector(tr_pj.rotation, {0.0,velocity.qv[joint.dof_index], 0.0});
    }
    if (joint.type == JointType::REVOLUTE_Z)
    {
        vel_result.angular = quat_rotate_vector(tr_pj.rotation, {0.0, 0.0, velocity.qv[joint.dof_index]});
    }
    else if (joint.type == JointType::PRISMATIC_X)
    {
        vel_result.linear = quat_rotate_vector(tr_pj.rotation, {velocity.qv[joint.dof_index], 0.0, 0.0});
    }
    else if (joint.type == JointType::PRISMATIC_Y)
    {
        vel_result.linear = quat_rotate_vector(tr_pj.rotation, {0.0, velocity.qv[joint.dof_index], 0.0});
    }
    else if (joint.type == JointType::PRISMATIC_Z)
    {
        vel_result.linear = quat_rotate_vector(tr_pj.rotation, {0.0, 0.0, velocity.qv[joint.dof_index]});
    }
    else if (joint.type == JointType::SPHERICAL)
    {
        const Vec3 angular_vel = {
            velocity.qv[joint.dof_index],
            velocity.qv[joint.dof_index + 1U],
            velocity.qv[joint.dof_index + 2U]
        };
        vel_result.angular = quat_rotate_vector(tr_pj.rotation, angular_vel);
    }
    else if (joint.type == JointType::CARTESIAN)
    {
        const MotionVector joint_vel = {
            {velocity.qv[joint.dof_index],      velocity.qv[joint.dof_index + 1U], velocity.qv[joint.dof_index + 2U]},
            {velocity.qv[joint.dof_index + 3U], velocity.qv[joint.dof_index + 4U], velocity.qv[joint.dof_index + 5U]}
        };
        vel_result = {
            quat_rotate_vector(tr_pj.rotation, joint_vel.angular),
            quat_rotate_vector(tr_pj.rotation, joint_vel.linear)
        };
    }
    else
    {
        // joint.type is JointType::FIXED
        // result is identity transform and zero velocity
    }

    return vel_result;
}

MotionVector joint_parent_child_acceleration(const Joint& joint, const JointPva& joint_pva)
{
    // alias constant transform to start of base
    const Transform& tr_pj = joint.parent_joint_transform;

    // initialize result
    MotionVector acceleration = {};

    // determine acceleration from joint motion
    if (joint.type == JointType::REVOLUTE_X)
    {
        acceleration.angular = quat_rotate_vector(tr_pj.rotation,{joint_pva.acceleration.qv[joint.dof_index], 0.0, 0.0});
    }
    if (joint.type == JointType::REVOLUTE_Y)
    {
        acceleration.angular = quat_rotate_vector(tr_pj.rotation,{0.0, joint_pva.acceleration.qv[joint.dof_index], 0.0});
    }
    if (joint.type == JointType::REVOLUTE_Z)
    {
        acceleration.angular = quat_rotate_vector(tr_pj.rotation,{0.0, 0.0, joint_pva.acceleration.qv[joint.dof_index]});
    }
    else if (joint.type == JointType::PRISMATIC_X)
    {
        acceleration.linear = quat_rotate_vector(tr_pj.rotation, {joint_pva.acceleration.qv[joint.dof_index], 0.0, 0.0});
    }
    else if (joint.type == JointType::PRISMATIC_Y)
    {
        acceleration.linear = quat_rotate_vector(tr_pj.rotation, {0.0, joint_pva.acceleration.qv[joint.dof_index], 0.0});
    }
    else if (joint.type == JointType::PRISMATIC_Z)
    {
        acceleration.linear = quat_rotate_vector(tr_pj.rotation, {0.0, 0.0, joint_pva.acceleration.qv[joint.dof_index]});
    }
    else if (joint.type == JointType::SPHERICAL)
    {
        const Vec3 angular_acc = {
            joint_pva.acceleration.qv[joint.dof_index],
            joint_pva.acceleration.qv[joint.dof_index + 1U],
            joint_pva.acceleration.qv[joint.dof_index + 2U]
        };
        acceleration.angular = quat_rotate_vector(tr_pj.rotation, angular_acc);
    }
    else if (joint.type == JointType::CARTESIAN)
    {
        const MotionVector joint_acc = {
            {joint_pva.acceleration.qv[joint.dof_index],      joint_pva.acceleration.qv[joint.dof_index + 1U],
             joint_pva.acceleration.qv[joint.dof_index + 2U]},
            {joint_pva.acceleration.qv[joint.dof_index + 3U], joint_pva.acceleration.qv[joint.dof_index + 4U],
             joint_pva.acceleration.qv[joint.dof_index + 5U]}
        };
        acceleration = {
            quat_rotate_vector(tr_pj.rotation, joint_acc.angular),
            quat_rotate_vector(tr_pj.rotation, joint_acc.linear)
        };
    }
    else
    {
        // result is identity transform, zero velocity, and zero acceleration
    }

    return acceleration;
}

CartesianPva joint_parent_child_pva(const Joint& joint, const JointPva& joint_pva)
{
    return {
        joint_parent_child_transform(joint, joint_pva.position), joint_parent_child_velocity(joint, joint_pva.velocity),
        joint_parent_child_acceleration(joint, joint_pva)};
}

} // namespace fsb
