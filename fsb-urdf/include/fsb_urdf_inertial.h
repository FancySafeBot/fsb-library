
#pragma once

#include "fsb_types.h"
#include "fsb_urdf_error.h"
#include "fsb_body.h"
#include <string>
#include <tinyxml2.h>

namespace fsb::urdf
{

/**
 * @addtogroup UrdfTopic
 * @{
 */

/**
 * @brief Parse inertial element from URDF file.
 *
 * This function parses the inertial element of a body in a URDF file.
 * An example xml element is:
 *
 * ```xml
 * <inertial>
 *   <mass value="1.0"/>
 *   <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
 * </inertial>
 * ```
 *
 * @param[in] fname File name of the URDF file
 * @param[in] body_name Body name to which the inertial element belongs
 * @param[in] inertial_xml Pointer to the inertial XML element
 * @param[out] body_mass Mass of the body
 * @param[out] err Error object to store any parsing errors
 * @return Inertia tensor
 */
Inertia urdf_parse_inertia_mass(
    const std::string& fname, const std::string& body_name, const tinyxml2::XMLElement* inertial_xml, real_t& body_mass,
    UrdfError& err);

/**
 * @}
 */

} // namespace fsb
