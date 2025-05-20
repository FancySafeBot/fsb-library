#include <cmath>
#include <cstddef>
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_configuration.h"
#include "fsb_joint.h"
#include "fsb_motion.h"
#include "fsb_quaternion.h"
#include "fsb_types.h"

namespace fsb
{

static BodyTreeError set_joint_coordinates(
    const JointType joint_type, size_t& num_coordinates, size_t& num_dofs, size_t& joint_coord_index,
    size_t& joint_dof_index)
{
    auto err = BodyTreeError::SUCCESS;
    if (joint_type != JointType::FIXED)
    {
        size_t joint_coords = 0U;
        size_t joint_dofs = 0U;
        if ((joint_type == JointType::REVOLUTE_X) ||
            (joint_type == JointType::REVOLUTE_Y) ||
            (joint_type == JointType::REVOLUTE_Z) ||
            (joint_type == JointType::PRISMATIC_X) ||
            (joint_type == JointType::PRISMATIC_Y) ||
            (joint_type == JointType::PRISMATIC_Z))
        {
            joint_coords = 1U;
            joint_dofs = 1U;
        }
        else if (joint_type == JointType::PLANAR)
        {
            joint_coords = 3U;
            joint_dofs = 3U;
        }
        else if (joint_type == JointType::SPHERICAL)
        {
            joint_coords = 4U;
            joint_dofs = 3U;
        }
        else if (joint_type == JointType::CARTESIAN)
        {
            joint_coords = 7U;
            joint_dofs = 6U;
        }
        else
        {
            // all joint types accounted for
        }

        // check max joint coordinates and dofs
        if ((num_coordinates + joint_coords) > MaxSize::coordinates)
        {
            err = BodyTreeError::MAX_JOINT_COORDINATES_REACHED;
        }
        else if ((num_dofs + joint_dofs) > MaxSize::dofs)
        {
            err = BodyTreeError::MAX_JOINT_DOFS_REACHED;
        }
        else
        {
            // set index for coordinates and dofs
            joint_coord_index = num_coordinates;
            joint_dof_index = num_dofs;
            // increment total
            num_coordinates += joint_coords;
            num_dofs += joint_dofs;
        }
    }
    return err;
}

BodyTreeError BodyTree::validate_massless_body_input(const size_t parent_body_index) const
{
    auto err = BodyTreeError::SUCCESS;
    if (m_num_bodies >= MaxSize::bodies)
    {
        err = BodyTreeError::MAX_BODIES_REACHED;
    }
    else if (m_num_joints >= MaxSize::joints)
    {
        err = BodyTreeError::MAX_JOINTS_REACHED;
    }
    else if (parent_body_index >= m_num_bodies)
    {
        err = BodyTreeError::PARENT_BODY_NONEXISTENT;
    }
    else
    {
        // no error
    }
    return err;
}

BodyTreeError BodyTree::validate_add_body_input(
    const size_t parent_body_index, const JointType joint_type, const Body& body) const
{
    auto err = BodyTreeError::SUCCESS;
    if (m_num_bodies >= MaxSize::bodies)
    {
        err = BodyTreeError::MAX_BODIES_REACHED;
    }
    else if (m_num_joints >= MaxSize::joints)
    {
        err = BodyTreeError::MAX_JOINTS_REACHED;
    }
    else if (parent_body_index >= m_num_bodies)
    {
        err = BodyTreeError::PARENT_BODY_NONEXISTENT;
    }
    else if ((joint_type != JointType::FIXED) && (body.mass_props.mass < FSB_TOL))
    {
        err = BodyTreeError::MASS_ZERO;
    }
    else if (
        (joint_type != JointType::FIXED)
        && ((fabs(body.mass_props.inertia.ixx) < FSB_TOL) || (fabs(body.mass_props.inertia.iyy) < FSB_TOL)
            || (fabs(body.mass_props.inertia.izz) < FSB_TOL)))
    {
        err = BodyTreeError::INERTIA_ZERO;
    }
    else if (
        (joint_type == JointType::FIXED) && (body.mass_props.mass < FSB_TOL)
        && ((body.mass_props.inertia.ixx != 0.0) || (body.mass_props.inertia.iyy != 0.0)
            || (body.mass_props.inertia.izz != 0.0) || (body.mass_props.inertia.ixy != 0.0)
            || (body.mass_props.inertia.ixz != 0.0) || (body.mass_props.inertia.iyz != 0.0)))
    {
        err = BodyTreeError::MASS_ZERO_WITH_INERTIA;
    }
    else if (!body_validate_inertia_is_pd(body.mass_props.inertia))
    {
        err = BodyTreeError::INERTIA_NOT_POS_DEF;
    }
    else
    {
        // no error
    }
    return err;
}

size_t BodyTree::add_massless_body(
    const size_t parent_body_index, const JointType joint_type, const Transform& parent_joint_transform,
    const MotionVector& origin_offset, BodyTreeError& err)
{
    size_t added_body_index = 0U;
    err = validate_massless_body_input(parent_body_index);
    if (err == BodyTreeError::SUCCESS)
    {
        // get joint coordinates from joint type
        size_t joint_coord_index = 0U;
        size_t joint_dof_index = 0U;
        err = set_joint_coordinates(joint_type, m_num_coordinates, m_num_dofs, joint_coord_index, joint_dof_index);
        if (err == BodyTreeError::SUCCESS)
        {
            // body and joint index to add
            const size_t joint_index = m_num_joints;
            const size_t body_index = m_num_bodies;
            // assign joint
            Joint &new_joint = m_joints[joint_index];
            new_joint.type = joint_type;
            new_joint.nominal_parent_joint_transform = parent_joint_transform;
            quat_normalize(new_joint.nominal_parent_joint_transform.rotation);
            // copy nominal as actual transform
            new_joint.parent_joint_transform = coord_transform_apply_body_offset(new_joint.nominal_parent_joint_transform, origin_offset);
            new_joint.coord_index = joint_coord_index;
            new_joint.dof_index = joint_dof_index;
            // set parent and child body indexes
            new_joint.parent_body_index = parent_body_index;
            new_joint.child_body_index = body_index;
            // add body
            Body &new_body = m_bodies[body_index];
            new_body.origin_offset = origin_offset;
            new_body.joint_index = joint_index;
            // parent body unset leaf status
            m_bodies[parent_body_index].is_leaf = false;
            new_body.is_leaf= true;
            // increment joint and body
            ++m_num_joints;
            ++m_num_bodies;
            // body indexed return value
            added_body_index = body_index;
        }
    }
    return added_body_index;
}

size_t BodyTree::add_body(
    const size_t parent_body_index, const JointType joint_type, const Transform& parent_joint_transform,
    const Body& body, BodyTreeError& err)
{
    size_t added_body_index = 0U;
    err = validate_add_body_input(parent_body_index, joint_type, body);
    if (err == BodyTreeError::SUCCESS)
    {
        // get joint coordinates from joint type
        size_t joint_coord_index = 0U;
        size_t joint_dof_index = 0U;
        err = set_joint_coordinates(joint_type, m_num_coordinates, m_num_dofs, joint_coord_index, joint_dof_index);
        if (err == BodyTreeError::SUCCESS)
        {
            // body and joint index to add
            const size_t joint_index = m_num_joints;
            const size_t body_index = m_num_bodies;
            // assign joint
            Joint &new_joint = m_joints[joint_index];
            new_joint.type = joint_type;
            new_joint.nominal_parent_joint_transform = parent_joint_transform;
            quat_normalize(new_joint.nominal_parent_joint_transform.rotation);
            // copy nominal as actual transform
            new_joint.parent_joint_transform = coord_transform_apply_body_offset(new_joint.nominal_parent_joint_transform, body.origin_offset);
            new_joint.coord_index = joint_coord_index;
            new_joint.dof_index = joint_dof_index;
            // set parent and child body indexes
            new_joint.parent_body_index = parent_body_index;
            new_joint.child_body_index = body_index;
            // add body
            Body &new_body = m_bodies[body_index];
            new_body.origin_offset = body.origin_offset;
            new_body.mass_props = body.mass_props;
            new_body.principal_inertia = body.principal_inertia;
            new_body.joint_index = joint_index;
            // parent body leaf status
            m_bodies[parent_body_index].is_leaf = false;
            new_body.is_leaf= true;
            // increment joint and body
            ++m_num_joints;
            ++m_num_bodies;
            // body indexed return value
            added_body_index = body_index;
        }
    }
    return added_body_index;
}

size_t BodyTree::get_body_dofs(const size_t body_index, BodyTreeError& err) const
{
    size_t dofs = 0U;
    if (body_index < m_num_bodies)
    {
        const Joint& joint = m_joints[m_bodies[body_index].joint_index];
        JointType joint_type = joint.type;
        size_t parent_index = joint.parent_body_index;
        size_t dof_index = joint.dof_index;
        while ((joint_type == JointType::FIXED) && (parent_index != base_index))
        {
            const Joint& parent_joint = m_joints[m_bodies[parent_index].joint_index];
            joint_type = parent_joint.type;
            parent_index = parent_joint.parent_body_index;
            dof_index = parent_joint.dof_index;
        }
        switch (joint_type)
        {
            case JointType::REVOLUTE_X:
            case JointType::REVOLUTE_Y:
            case JointType::REVOLUTE_Z:
            case JointType::PRISMATIC_X:
            case JointType::PRISMATIC_Y:
            case JointType::PRISMATIC_Z:
            {
                dofs = dof_index + 1U;
                break;
            }
            case JointType::SPHERICAL:
            case JointType::PLANAR:
            {
                dofs = dof_index + 3U;
                break;
            }
            case JointType::CARTESIAN:
            {
                dofs = dof_index + 6U;
                break;
            }
            case JointType::FIXED:
            default:
            {
                // FIXED or unknown: do nothing
                break;
            }
        }
        err = BodyTreeError::SUCCESS;
    }
    else
    {
        err = BodyTreeError::BODY_NOT_IN_TREE;
    }

    return dofs;
}

} // namespace fsb
