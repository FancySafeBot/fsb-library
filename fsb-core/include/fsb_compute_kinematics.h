
#ifndef FSB_COMPUTE_KINEMATICS_H
#define FSB_COMPUTE_KINEMATICS_H

#include <cstdint>
#include <cstddef>

#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_joint.h"
#include "fsb_kinematics.h"
#include "fsb_jacobian.h"
#include "fsb_motion.h"

namespace fsb
{

/**
 * @defgroup TopicInterface Forward Kinematics Computation
 * @brief Standalone forward kinematics computation module
 *
 * @{
 */

/**
 * @brief Interface error codes
 *
 */
enum class ComputeKinematicsError : uint8_t
{
    /**
     * @brief No error
     */
    SUCCESS = 0,
    /**
     * @brief Body tree is invalid
     */
    INVALID_BODY_TREE = 1
};

/**
 * @brief Compute forward kinematics
 */
class ComputeKinematics
{
public:
    ComputeKinematics() = default;

    /**
     * @brief Initialize the computation interface with a body tree
     *
     * @param[in] tree Body tree
     * @return Error code
     */
    ComputeKinematicsError initialize(const BodyTree& tree);

    /**
     * @brief Compute forward kinematics
     *
     * @param[in] joint_position Joint position vector
     * @param[out] cartesian Output list of body cartesian poses
     */
    void compute_forward_kinematics_pose(
        const JointSpacePosition& joint_position, BodyCartesianPva& cartesian) const;

    /**
     * @brief Compute forward kinematics with optional velocity and acceleration
     *
     * @param[in] opt Pose, velocity or acceleration selection
     * @param[in] joint Joint position, velocity and acceleration
     * @param[out] cartesian Output Cartesian pose, velocity and acceleration of all bodies
     */
    void compute_forward_kinematics(
        ForwardKinematicsOption opt, const JointPva& joint, BodyCartesianPva& cartesian) const;

    /**
     * @brief Compute forward kinematics with optional velocity and acceleration
     *
     * @param[in] opt Pose, velocity or acceleration selection
     * @param[in] joint Joint position, velocity and acceleration
     * @param[in] base Base pose, velocity and acceleration
     * @param[out] cartesian Output Cartesian pose, velocity and acceleration of all bodies
     */
    void compute_forward_kinematics_with_base(
        ForwardKinematicsOption opt, const JointPva& joint, const CartesianPva& base,
        BodyCartesianPva& cartesian) const;

    /**
     * @brief Compute Jacobian matrix for a single body in tree
     *
     * @param[in] body_index
     * @param[in] cartesian
     * @param[out] jacobian
     * @return Jacobian error code
     */
    JacobianError compute_jacobian(
        size_t body_index, const BodyCartesianPva& cartesian, Jacobian& jacobian) const;

    /**
     * @brief Get the number of bodies in the body tree
     *
     * @return Number of bodies
     */
    [[nodiscard]] size_t get_num_bodies() const
    {
        return m_body_tree.get_num_bodies();
    }

    /**
     * @brief Get the number of coordinates (joint position vector size) in the body tree
     *
     * @return Number of joint coordinates
     */
    [[nodiscard]] size_t get_num_coordinates() const
    {
        return m_body_tree.get_num_coordinates();
    }

private:
    BodyTree m_body_tree;
};

inline ComputeKinematicsError ComputeKinematics::initialize(const BodyTree& tree)
{
    m_body_tree = tree;
    return ComputeKinematicsError::SUCCESS;
}

inline void ComputeKinematics::compute_forward_kinematics_pose(
    const JointSpacePosition& joint_position, BodyCartesianPva& cartesian) const
{
    const JointPva joint = {joint_position, {}, {}};
    compute_forward_kinematics(ForwardKinematicsOption::POSE, joint, cartesian);
}

inline void ComputeKinematics::compute_forward_kinematics(
    const ForwardKinematicsOption opt, const JointPva& joint, BodyCartesianPva& cartesian) const
{
    forward_kinematics(m_body_tree, joint, {}, opt, cartesian);
}

inline void ComputeKinematics::compute_forward_kinematics_with_base(
    const ForwardKinematicsOption opt, const JointPva& joint, const CartesianPva& base,
    BodyCartesianPva& cartesian) const
{
    forward_kinematics(m_body_tree, joint, base, opt, cartesian);
}

inline JacobianError ComputeKinematics::compute_jacobian(
    const size_t body_index, const BodyCartesianPva& cartesian, Jacobian& jacobian) const
{
    return calculate_jacobian(body_index, m_body_tree, cartesian, jacobian);
}

/**
 * @}
 */

} // namespace fsb

#endif
