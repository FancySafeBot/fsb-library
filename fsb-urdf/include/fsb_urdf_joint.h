
#pragma once

#include "fsb_motion.h"
#include "fsb_types.h"
#include "fsb_urdf_error.h"
#include "fsb_joint.h"

#include <string>
#include <tinyxml2.h>

namespace fsb::urdf
{

/**
 * @addtogroup UrdfTopic
 * @{
 */


struct UrdfJointLimits
{
    /** Lower position limit */
    real_t lower_position = 0.0;

    /** Upper position limit */
    real_t upper_position = 0.0;

    /** Maximum velocity allowed */
    real_t max_velocity = 0.0;

    /** Whether position limits are set */
    bool set = false;
};

/**
 * @brief Structure representing a URDF joint.
 */
struct UrdfJoint
{
    /** Name of the joint */
    std::string joint_name;

    /** Type of joint */
    JointType joint_type = JointType::FIXED;

    /** Name of the parent link */
    std::string parent_name;

    /** Name of the child link */
    std::string child_name;

    /** Transform from parent to child link */
    Transform parent_child_transform = {};

    /** Joint limits */
    UrdfJointLimits limits = {};

    /** @brief reversed direction of the joint axis */
    bool reversed = false;
};

/**
 * @brief Parse a URDF joint element.
 *
 * An example xml snippet with a joint element is shown below.
 *
 * ```xml
 * <joint name="jointname" type="revolute">
 *   <parent link="parent_linkname"/>
 *   <child link="child_linkname"/>
 *   <origin xyz="0 0 0" rpy="0 0 0"/>
 *   <axis xyz="0 0 1"/>
 * </joint>
 * ```
 *
 * @param[in] fname Name of the file being parsed
 * @param[in] joint_xml XML element representing the joint
 * @param[out] err Error status returned after parsing
 * @return Parsed URDF joint
 */
UrdfJoint urdf_parse_joint(const std::string& fname, const tinyxml2::XMLElement* joint_xml, UrdfError& err);

/**
 * @}
 */

} // namespace fsb
