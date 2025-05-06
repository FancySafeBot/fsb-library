
#ifndef FSB_JOINT_H
#define FSB_JOINT_H

#include <array>
#include <cstddef>
#include "fsb_configuration.h"
#include "fsb_types.h"
#include "fsb_motion.h"

namespace fsb
{

/**
 * @defgroup Joint Joint Definitions and Kinematics
 * @brief Robot joint types, transforms, and model object.
 *
 * The @c Joint data container is used in @c RigidBodyTree as part of the robot model.
 *
 * @{
 */

/**
 * @brief Joint type
 */
enum class JointType : uint8_t
{
    /** @brief Fixed joint */
    FIXED = 0,
    /** @brief Revolute joint about the x-axis */
    REVOLUTE_X = 1,
    /** @brief Revolute joint about the y-axis */
    REVOLUTE_Y = 2,
    /** @brief Revolute joint about the z-axis */
    REVOLUTE_Z = 3,
    /** @brief Prismatic joint along x-axis */
    PRISMATIC_X = 4,
    /** @brief Prismatic joint along y-axis */
    PRISMATIC_Y = 5,
    /** @brief Prismatic joint along z-axis */
    PRISMATIC_Z = 6,
    /** @brief Spherical joint */
    SPHERICAL = 7,
    /** @brief 6 DoF Cartesian joint */
    CARTESIAN = 8,
    /** @brief Planar joint in XY plane */
    PLANAR = 9
};

/**
 * @brief Joint space position in generalized coordinates
 */
struct JointSpacePosition
{
    /**
     * @brief Joint space position data array
     */
    std::array<real_t, MaxSize::coordinates> q;
};

/**
 * @brief Joint space differential of generalized coordinates
 */
struct JointSpace
{
    /**
     * @brief Joint space differential data array
     */
    std::array<real_t, MaxSize::dofs> qv;
};

/**
 * Joint position, velocity and acceleration
 */
struct JointPva
{
    /**
     * @brief Joint position
     */
    JointSpacePosition position;
    /**
     * @brief Joint velocity
     */
    JointSpace velocity;
    /**
     * @brief Joint acceleration
     */
    JointSpace acceleration;
};

/**
 * @brief Joint definition structure
 */
struct Joint
{
    /**
     * @brief Joint type
     */
    JointType type;
    /**
     * @brief Joint pose with respect to the parent body with offset from parent body
     */
    Transform parent_joint_transform;
    /**
     * @brief Joint pose with respect to the parent body without offset
     */
    Transform nominal_parent_joint_transform;
    /**
     * @brief Index of model parent body to which the joint is attached
     */
    size_t parent_body_index;
    /**
     * @brief Index of model child body to which the joint is connected
     */
    size_t child_body_index;
    /**
     * @brief Index of the joint's first generalized coordinate in the model joint space position
     * vector @c JointSpacePosition
     */
    size_t coord_index;
    /**
     * @brief Index of the joint's first differential generalized coordinate in the model joint
     * space vector @c JointSpace
     */
    size_t dof_index;
};

/**
 * @brief Get transform of child body with respect to parent body
 *
 * @param[in] joint Joint definition
 * @param[in] position Joint space position
 * @return Coordinate transform of child body with respect to parent body
 */
Transform joint_parent_child_transform(const Joint& joint, const JointSpacePosition& position);

/**
 * @brief Get the Cartesian space velocity of the child body with respect to the parent body
 *
 * @param[in] joint Joint definition
 * @param[in] velocity Joint space velocity
 * @return Cartesian space velocity of the child body with respect to the parent body
 */
MotionVector joint_parent_child_velocity(const Joint& joint, const JointSpace& velocity);

/**
 * @brief Get the Cartesian space acceleration of the child body with respect to the parent body
 *
 * @param[in] joint Joint definition
 * @param[in] joint_pva Joint space position, velocity and acceleration
 * @return Cartesian space acceleration of the child body with respect to the parent body
 */
MotionVector joint_parent_child_acceleration(const Joint& joint, const JointPva& joint_pva);

/**
 * @brief Get the Cartesian space pose, velocity and acceleration of the child body with respect to
 * the parent body
 *
 * @param[in] joint Joint definition
 * @param[in] joint_pva Joint space position, velocity and acceleration
 * @return Cartesian space pose, velocity and acceleration of the child body with respect to the
 * parent body
 */
CartesianPva joint_parent_child_pva(const Joint& joint, const JointPva& joint_pva);

/**
 * @brief Validate joint definition
 *
 * @param[in] joint Joint definition
 * @return true if valid
 * @return false if invalid
 */
bool joint_validate(const Joint& joint);

/**
 * @}
 */

} // namespace fsb

#endif // FSB_JOINT_H
