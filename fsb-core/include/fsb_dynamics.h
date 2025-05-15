#ifndef FSB_DYNAMICS_H
#define FSB_DYNAMICS_H

#include <array>

#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_configuration.h"
#include "fsb_joint.h"
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup Dynamics Robot Dynamics
 * @brief Robot model dynamics
 *
 * @{
 */

/**
 * @brief Force and torque vector
 */
struct ForceVector
{
    Vec3 torque; ///< Torque vector
    Vec3 force; ///< Force vector
};

/**
 * @brief Container of Forces and Torques for a list of bodies
 */
struct BodyForce
{
    /**
     * @brief List of bodies with Cartesian pose, velocity, and acceleration.
     */
    std::array<ForceVector, MaxSize::bodies> body;
};

/**
 * @brief Inverse dynamics to find joint torques based on external forces and motion of bodies.
 *
 * @param body_tree Body tree
 * @param cartesian_motion Cartesian motion of bodies
 * @param external_force External forces applied to bodies
 * @param body_force Forces and torque applied to bodies acting at each joint
 * @return Joint torque vector resulting from dynamics
 */
JointSpace inverse_dynamics(
    const BodyTree& body_tree, const BodyCartesianPva& cartesian_motion, const BodyForce& external_force,
    BodyForce& body_force);

/**
 * @}
 */

}

#endif //FSB_DYNAMICS_H
