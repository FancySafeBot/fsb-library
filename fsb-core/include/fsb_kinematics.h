
#ifndef FSB_KINEMATICS_H
#define FSB_KINEMATICS_H

#include <cstdint>
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_joint.h"
#include "fsb_motion.h"

namespace fsb
{

/**
 * @defgroup Kinematics Robot Kinematics
 * @brief Functions to compute robot model kinematics
 *
 * @{
 */

/**
 * @brief Forward kinematics options
 */
enum class ForwardKinematicsOption: uint8_t
{
    /**
     * @brief Calculate Cartesian pose from joint position
     */
    POSE = 0,
    /**
     * @brief Calculate Cartesian pose and velocity from joint position and velocity
     */
    POSE_VELOCITY = 1,
    /**
     * @brief Calculate Cartesian pose, velocity and acceleration from
     *        joint position, velocity, and acceleration
     */
    POSE_VELOCITY_ACCELERATION = 2
};

/**
 * @brief Compute forward kinematics for all bodies in tree
 *
 * @param[in] body_tree Body tree with body and joint definitions
 * @param[in] joint_pva Joint position, velocity and acceleration
 * @param[in] base_pva Base pose, velocity and acceleration
 * @param[in] opt Compute options
 * @param[out] body_cartesian Output Cartesian pose, velocity and acceleration of all bodies
 */
void forward_kinematics(
    const BodyTree& body_tree, const JointPva& joint_pva, const CartesianPva& base_pva,
    ForwardKinematicsOption opt, BodyCartesianPva& body_cartesian);

/**
 * @brief Add offset to joint position
 *
 * @param[in] body_tree Body tree with body and joint definitions
 * @param[in] joint_position Joint position
 * @param[in] joint_offset Joint offset
 * @return Joint position with offset added
 */
JointSpacePosition joint_add_offset(
    const BodyTree& body_tree, const JointSpacePosition& joint_position, const JointSpace& joint_offset);

/**
 * @brief Get difference between joint positions
 *
 * @param[in] body_tree Body tree with body and joint definitions
 * @param[in] joint_position_a Joint position a
 * @param[in] joint_position_b Joint position b
 * @return Joint position difference
 */
JointSpace joint_difference(
    const BodyTree& body_tree, const JointSpacePosition& joint_position_a, const JointSpacePosition& joint_position_b);

/**
 * @brief Compute motion of center of mass for all bodies in tree.
 *
 * @param body_tree Body tree with link definitions including center of mass
 * @param body_cartesian Computed kinematics of bodies in tree
 * @param com_cartesian Output motion of center of mass for all bodies in tree
 */
void body_com_kinematics(
    const BodyTree& body_tree, const BodyCartesianPva& body_cartesian,
    BodyCartesianPva& com_cartesian);

/**
 * @}
 */

} // namespace fsb

#endif
