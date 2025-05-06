
#ifndef FSB_BODY_TREE_H
#define FSB_BODY_TREE_H

#include <array>
#include <cstddef>
#include "fsb_configuration.h"
#include "fsb_body.h"
#include "fsb_joint.h"
#include "fsb_motion.h"

namespace fsb
{

/**
 * @defgroup TopicBodyTree Body Tree
 * @brief Rigid body tree containing geometry and mass properties of robot model
 *
 * @{
 */

/**
 * @brief Error list for body tree operations.
 */
enum class BodyTreeError : uint8_t
{
    SUCCESS = 0,
    /**
     * @brief Index does not point to an existing body in tree.
     */
    BODY_NOT_IN_TREE = 1,
    /**
     * @brief Index does not point to an existing joint in tree.
     */
    JOINT_NOT_IN_TREE = 2,
    /**
     * @brief No more bodies can be added to the body tree. Maximum is specified by @c
     * MaxSize::bodies
     */
    MAX_BODIES_REACHED = 3,
    /**
     * @brief Parent body index does not point to an existing body in tree.
     */
    PARENT_BODY_NONEXISTENT = 4,
    /**
     * @brief No more joints can be added to the body tree. Maximum is specified by @c
     * MaxSize::joints
     */
    MAX_JOINTS_REACHED = 5,
    /**
     * @brief Joint requires more coordinates than is available. Maximum is specified by @c
     * MaxSize::coordinates
     */
    MAX_JOINT_COORDINATES_REACHED = 6,
    /**
     * @brief Joint requires more DoFs than is available. Maximum is specified by @c MaxSize::dofs
     */
    MAX_JOINT_DOFS_REACHED = 7,
    /**
     * @brief Body mass is zero with a non-fixed parent joint
     */
    MASS_ZERO = 8,
    /**
     * @brief At least one of the inertia principal components are zero with a non-fixed parent
     * joint
     */
    INERTIA_ZERO = 9,
    /**
     * @brief  Body mass is set to zero but with non-zero inertia for a fixed parent joint.
     */
    MASS_ZERO_WITH_INERTIA = 10,
    /**
     * @brief Inertia is not positive definite.
     */
    INERTIA_NOT_POS_DEF = 11
};

/**
 * @brief Body tree description of a kinematic chain
 *
 */
class BodyTree
{
public:
    /**
     * @brief Construct a new BodyTree object
     */
    BodyTree() = default;

    /** @brief Index of base body is always 0 */
    static constexpr size_t base_index = 0U;

    /**
     * @brief Add a body to the body tree
     *
     * @param parent_body_index Index of parent body
     * @param joint_type Joint type
     * @param parent_joint_transform Transform of joint with respect to parent body
     * @param body Body to add to tree
     * @param err Error code
     * @return Index of added body
     */
    size_t add_body(
        size_t parent_body_index, JointType joint_type, const Transform& parent_joint_transform,
        const Body& body, BodyTreeError& err);

    size_t add_massless_body(
        size_t parent_body_index, JointType joint_type, const Transform& parent_joint_transform,
        const MotionVector& origin_offset, BodyTreeError& err);

    /**
     * @brief Get a joint object in the body tree
     *
     * @param[in] joint_index Joint index
     * @param[out] err Error code
     * @return Joint object
     */
    Joint get_joint(size_t joint_index, BodyTreeError& err) const;

    /**
     * @brief Get body object from tree
     *
     * @param[in] body_index Body index
     * @param[out] err Error code
     * @return Body object
     */
    Body get_body(size_t body_index, BodyTreeError& err) const;

    /**
     * @brief Set the body mass properties object
     *
     * @param body_index Body index
     * @param mass_props Mass properties to set
     * @return Error code
     */
    BodyTreeError set_body_mass_props(size_t body_index, const MassProps& mass_props);

    /**
     * @brief Set the body transform offset
     *
     * @param body_index Body index
     * @param origin_offset Origin offset transform
     * @return Error code
     */
    BodyTreeError set_body_origin_offset(size_t body_index, const MotionVector& origin_offset);

    /**
     * @brief Set the parent joint transform object
     *
     * @param body_index Body index
     * @param parent_joint_transform Nominal transform of body's parent joint
     * @return Error code
     */
    BodyTreeError
    set_parent_joint_transform(size_t body_index, const Transform& parent_joint_transform);

    /**
     * @brief Get the number of bodies in tree
     *
     * @return Number of bodies
     */
    [[nodiscard]] size_t get_num_bodies() const
    {
        return m_num_bodies;
    }

    /**
     * @brief Get the number of body tree degrees of freedom
     *
     * @return Degrees of freedom
     */
    [[nodiscard]] size_t get_num_dofs() const
    {
        return m_num_dofs;
    }

    /**
     * @brief Get the number of generalized position coordinates
     *
     * @return Number of generalized position coordinates
     */
    [[nodiscard]] size_t get_num_coordinates() const
    {
        return m_num_coordinates;
    }

private:
    [[nodiscard]] BodyTreeError validate_massless_body_input(size_t parent_body_index) const;
    [[nodiscard]] BodyTreeError
    validate_add_body_input(size_t parent_body_index, JointType joint_type, const Body& body) const;

    std::array<Body, MaxSize::bodies>  m_bodies = {};
    std::array<Joint, MaxSize::joints> m_joints = {};

    size_t m_num_coordinates = 0U;
    size_t m_num_dofs = 0U;
    size_t m_num_bodies = 1U;
    size_t m_num_joints = 0U;
};

// inline methods

inline Joint BodyTree::get_joint(const size_t joint_index, BodyTreeError& err) const
{
    Joint joint = {};
    if (joint_index < m_num_joints)
    {
        joint = m_joints[joint_index];
        err = BodyTreeError::SUCCESS;
    }
    else
    {
        err = BodyTreeError::JOINT_NOT_IN_TREE;
    }
    return joint;
}

inline Body BodyTree::get_body(const size_t body_index, BodyTreeError& err) const
{
    Body body = {};
    if (body_index < m_num_bodies)
    {
        body = m_bodies[body_index];
        err = BodyTreeError::SUCCESS;
    }
    else
    {
        err = BodyTreeError::BODY_NOT_IN_TREE;
    }
    return body;
}

inline BodyTreeError
BodyTree::set_body_mass_props(const size_t body_index, const MassProps& mass_props)
{
    auto err = BodyTreeError::SUCCESS;
    if (body_index < m_num_bodies)
    {
        m_bodies[body_index].mass_props = mass_props;
        err = BodyTreeError::SUCCESS;
    }
    else
    {
        err = BodyTreeError::BODY_NOT_IN_TREE;
    }
    return err;
}

inline BodyTreeError
BodyTree::set_body_origin_offset(const size_t body_index, const MotionVector& origin_offset)
{
    auto err = BodyTreeError::SUCCESS;
    if (body_index < m_num_bodies)
    {
        m_bodies[body_index].origin_offset = origin_offset;
        err = BodyTreeError::SUCCESS;
    }
    else
    {
        err = BodyTreeError::BODY_NOT_IN_TREE;
    }
    return err;
}

inline BodyTreeError BodyTree::set_parent_joint_transform(
    const size_t body_index, const Transform& parent_joint_transform)
{
    auto err = BodyTreeError::SUCCESS;
    if (body_index < m_num_bodies)
    {
        const size_t joint_index = m_bodies[body_index].joint_index;
        m_joints[joint_index].parent_joint_transform = parent_joint_transform;
        err = BodyTreeError::SUCCESS;
    }
    else
    {
        err = BodyTreeError::BODY_NOT_IN_TREE;
    }
    return err;
}

/**
 * @}
 */

} // namespace fsb

#endif
