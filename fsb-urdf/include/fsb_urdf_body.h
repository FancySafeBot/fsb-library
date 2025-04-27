
#pragma once

#include "fsb_body.h"
#include "fsb_motion.h"
#include "fsb_urdf_error.h"

#include <string>
#include <tinyxml2.h>

namespace fsb::urdf
{

/**
 * @addtogroup UrdfTopic
 * @{
 */

/**
 * @brief Link element data from URDF
 */
struct UrdfLink
{
    /** @brief Link name
     */
    std::string link_name;
    /**
    * @brief Transform offset of from nominal body origin.
    */
    MotionVector origin_offset = {};
    /**
     * @brief Mass properties of body
     */
    MassProps mass_props = {};
    /**
     * @brief Principal inertia of body
     */
    PrincipalInertia principal_inertia = {};
};

/**
 * @brief Parse link element from xml
 *
 * An example xml snippet with a link element is shown below.
 *
 * ```xml
 * <link name="linkname">
 *   <inertial>
 *     <origin xyz="0 0 0" rpy="0 0 0" />
 *     <mass value="1.0" />
 *     <inertia ixx="1.0" iyy="1.0" izz="1.0" ixy="0" ixz="0" iyz="0" />
 *   </inertial>
 *   <fsb:origin_offset xyz="0 0 0" rotvec="0 0 0" />
 * </link>
 * ```
 *
 * @param fname Filename being parsed
 * @param link_xml Link xml element to parse
 * @param err Error object
 * @return Link data from parsed link element
 */
UrdfLink urdf_parse_link(const std::string& fname, const tinyxml2::XMLElement* link_xml, UrdfError& err);

/**
 * @}
 */

} // namespace fsb
