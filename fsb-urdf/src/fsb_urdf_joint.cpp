
#include "fsb_urdf_error.h"
#include "fsb_urdf_utilities.h"
#include "fsb_urdf_joint.h"
#include "fsb_urdf_origin.h"

#include "fsb_types.h"
#include "fsb_motion.h"
#include "fsb_joint.h"

#include <cmath>
#include <string>
#include <tinyxml2.h>

namespace fsb::urdf
{

static UrdfJointLimits parse_joint_limits(
    const std::string& fname, const std::string& joint_name,
    const tinyxml2::XMLElement* joint_xml, const JointType& joint_type, UrdfError& err)
{
    UrdfJointLimits limits = {};
    const tinyxml2::XMLElement* joint_limits_xml = joint_xml->FirstChildElement("limit");

    if (joint_limits_xml != nullptr &&
        (joint_type == JointType::REVOLUTE_X ||
         joint_type == JointType::REVOLUTE_Y ||
         joint_type == JointType::REVOLUTE_Z ||
         joint_type == JointType::PRISMATIC_X ||
         joint_type == JointType::PRISMATIC_Y ||
         joint_type == JointType::PRISMATIC_Z))
    {
        // lower limit
        if (const char* lower = joint_limits_xml->Attribute("lower");
            lower != nullptr)
        {
            const auto lower_position_str = std::string(lower);
            const real_t lower_position = string_to_real(lower_position_str, err);
            if (!err.is_error())
            {
                limits.lower_position = lower_position;
                limits.set = true;
            }
            else
            {
                err = {UrdfErrorType::JOINT_LIMITS_PARSE_ERROR,
                               "Invalid joint '" + joint_name + "' limit lower='"
                                   + lower_position_str + "' in URDF file '" + fname + "'"};
            }
        }
        // upper limit
        if (const char* upper = joint_limits_xml->Attribute("upper");
            upper != nullptr)
        {
            const auto upper_position_str = std::string(upper);
            const real_t upper_position = string_to_real(upper_position_str, err);
            if (!err.is_error())
            {
                limits.upper_position = upper_position;
                limits.set = true;
            }
            else
            {
                err = {UrdfErrorType::JOINT_LIMITS_PARSE_ERROR,
                               "Invalid joint '" + joint_name + "' limit upper='"
                                   + upper_position_str + "' in URDF file '" + fname + "'"};
            }
        }
        // velocity limit
        if (const char* velocity = joint_limits_xml->Attribute("velocity");
            velocity != nullptr)
        {
            const auto max_velocity_str = std::string(velocity);
            const real_t max_velocity = string_to_real(max_velocity_str, err);
            if (!err.is_error())
            {
                limits.max_velocity = max_velocity;
            }
            else
            {
                err = {UrdfErrorType::JOINT_LIMITS_PARSE_ERROR,
                               "Invalid joint '" + joint_name + "' limit velocity='"
                                   + max_velocity_str + "' in URDF file '" + fname + "'"};
            }
        }
    }
    return limits;
}

static void parse_joint_axis(
    const std::string& fname, const std::string& joint_name,
    const tinyxml2::XMLElement* joint_axis_xml, JointType& joint_type, bool& reversed, UrdfError& err)
{
    reversed = false;
    if (joint_type == JointType::REVOLUTE_Z || joint_type == JointType::PRISMATIC_Z)
    {
        // axis xyz
        if (const char* joint_axis = joint_axis_xml->Attribute("xyz"); joint_axis != nullptr)
        {
            const auto joint_axis_str = std::string(joint_axis);
            Vec3       axis = string_to_vector(joint_axis_str, err);
            if (!err.is_error())
            {
                if (const real_t vec_norm = vector_norm(axis); vec_norm > FSB_TOL)
                {
                    const Vec3 axis_pos = vector_scale(1.0 / vec_norm, axis);
                    if ((axis_pos.x > (1.0 - FSB_TOL)) && (axis_pos.y < FSB_TOL) && (axis_pos.z < FSB_TOL))
                    {
                        joint_type
                            = (joint_type == JointType::REVOLUTE_Z ? JointType::REVOLUTE_X :
                                                                     JointType::PRISMATIC_X);
                        reversed = (axis.x < 0.0);
                    }
                    else if ((axis_pos.y > (1.0 - FSB_TOL)) && (axis_pos.x < FSB_TOL) && (axis_pos.z < FSB_TOL))
                    {
                        joint_type
                            = (joint_type == JointType::REVOLUTE_Z ? JointType::REVOLUTE_Y :
                                                                     JointType::PRISMATIC_Y);
                        reversed = (axis.y < 0.0);
                    }
                    else if ((axis_pos.z > (1.0 - FSB_TOL)) && (axis_pos.x < FSB_TOL) && (axis_pos.y < FSB_TOL))
                    {
                        // +z-axis joint type is valid
                        reversed = (axis.z < 0.0);
                    }
                    else
                    {
                        err
                            = {UrdfErrorType::JOINT_INVALID_AXIS,
                               "Invalid joint '" + joint_name + "' axis direction xyz='"
                                   + joint_axis_str + "' in URDF file '" + fname + "'"};
                    }
                }
            }
            else
            {
                err
                    = {UrdfErrorType::JOINT_INVALID_AXIS,
                       "Invalid joint '" + joint_name + "' axis direction xyz='" + joint_axis_str
                           + "' in URDF file '" + fname + "'"};
            }
        }
    }
}

static std::string urdf_parse_joint_name_type(
    const std::string& fname, const tinyxml2::XMLElement* joint_xml, JointType& joint_type,
    bool& reversed, UrdfError& err)
{
    std::string joint_name;

    // Attributes
    const char* joint_name_item = joint_xml->Attribute("name");
    const char* joint_type_item = joint_xml->Attribute("type");
    // Child elements
    const tinyxml2::XMLElement* joint_axis_xml = joint_xml->FirstChildElement("axis");

    if (joint_name_item == nullptr)
    {
        err
            = {UrdfErrorType::MISSING_NAME,
               "Joint name attribute is missing in URDF file '" + fname + "'"};
    }
    else if (joint_type_item == nullptr)
    {
        err = {UrdfErrorType::MISSING_JOINT_TYPE, "Joint type attribute is missing"};
    }

    if (!err.is_error())
    {
        joint_name = std::string(joint_name_item);
        if (const auto joint_type_string = std::string(joint_type_item);
            (joint_type_string == "revolute") || (joint_type_string == "continuous"))
        {
            joint_type = JointType::REVOLUTE_Z;
        }
        else if (joint_type_string == "prismatic")
        {
            joint_type = JointType::PRISMATIC_Z;
        }
        else if (joint_type_string == "fixed")
        {
            joint_type = JointType::FIXED;
        }
        else if (joint_type_string == "planar")
        {
            joint_type = JointType::PLANAR;
        }
        else if (joint_type_string == "spherical")
        {
            joint_type = JointType::SPHERICAL;
        }
        else if ((joint_type_string == "floating") || (joint_type_string == "cartesian"))
        {
            joint_type = JointType::CARTESIAN;
        }
        else
        {
            err
                = {UrdfErrorType::INVALID_JOINT_TYPE,
                   "Invalid joint type '" + joint_type_string + "' in URDF file '" + fname + "'"};
        }
    }

    if (!err.is_error() && (joint_axis_xml != nullptr))
    {
        parse_joint_axis(fname, joint_name, joint_axis_xml, joint_type, reversed, err);
    }

    return joint_name;
}

static Transform urdf_parse_joint_parent_child_transform(
    const std::string& fname, const std::string& joint_name, const tinyxml2::XMLElement* joint_xml,
    std::string& parent_name, std::string& child_name, UrdfError& err)
{
    Transform parent_child_transf = transform_identity();

    // child elements
    const tinyxml2::XMLElement* parent_xml = joint_xml->FirstChildElement("parent");
    const tinyxml2::XMLElement* child_xml = joint_xml->FirstChildElement("child");

    if (parent_xml == nullptr)
    {
        err = {
            UrdfErrorType::MISSING_JOINT_PARENT,
            "Joint '" + joint_name + "' <parent> element is missing in URDF file '" + fname + "'"};
    }
    else if (child_xml == nullptr)
    {
        err = {
            UrdfErrorType::MISSING_JOINT_CHILD,
            "Joint '" + joint_name + "' <child> element is missing in URDF file '" + fname + "'"};
    }

    // parent link name
    if (!err.is_error() && (parent_xml != nullptr))
    {
        if (const char* link_value = parent_xml->Attribute("link"); link_value == nullptr)
        {
            err
                = {UrdfErrorType::MISSING_PARENT_LINK_ATTRIBUTE,
                   "Missing link attribute in joint '" + joint_name
                       + "' <parent> element in URDF file '" + fname + "'"};
        }
        else
        {
            parent_name = std::string(link_value);
        }
    }

    // child link name
    if (!err.is_error() && (child_xml != nullptr))
    {
        if (const char* link_value = child_xml->Attribute("link"); link_value == nullptr)
        {
            err
                = {UrdfErrorType::MISSING_CHILD_LINK_ATTRIBUTE,
                   "Missing link attribute in joint '" + joint_name
                       + "' <child> element in URDF file '" + fname + "'"};
        }
        else
        {
            child_name = std::string(link_value);
        }
    }

    // parent child transform
    if (!err.is_error())
    {
        parent_child_transf = urdf_parse_origin(fname, joint_name, joint_xml, err);
    }

    return parent_child_transf;
}

UrdfJoint
urdf_parse_joint(const std::string& fname, const tinyxml2::XMLElement* joint_xml, UrdfError& err)
{
    UrdfJoint joint = {};
    joint.joint_name = urdf_parse_joint_name_type(fname, joint_xml, joint.joint_type, joint.reversed, err);
    if (!err.is_error())
    {
        joint.parent_child_transform = urdf_parse_joint_parent_child_transform(
            fname, joint.joint_name, joint_xml, joint.parent_name, joint.child_name, err);
    }

    if (!err.is_error())
    {
        joint.limits = parse_joint_limits(fname, joint.joint_name, joint_xml, joint.joint_type, err);
    }

    return joint;
}

} // namespace fsb::urdf
