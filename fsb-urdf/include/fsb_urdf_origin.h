
#pragma once

#include "fsb_urdf_error.h"
#include "fsb_motion.h"
#include <string>
#include <tinyxml2.h>

namespace fsb::urdf
{

/**
 * @addtogroup UrdfTopic
 * @{
 */

/**
 * @brief Parse first child origin element to get coordinate transform.
 *
 * Here is an example xml snippet with the child origin of an element
 *
 * ```xml
 * <element name="elname">
 *   <origin xyz="0 0 0" rpy="0 0 0" />
 * </element>
 * ```
 *
 * @param fname Name of file being parsed
 * @param el_name Parent element name containing origin
 * @param el_xml Parent element containing origin
 * @param err Error status returned after parsing
 * @return Coordinate transform of origin
 */
Transform urdf_parse_origin(const std::string& fname, const std::string& el_name, const tinyxml2::XMLElement* el_xml, UrdfError& err);

/**
 * @brief Parse first child origin offset element.
 *
 * The origin offset is a custom element for the Fancy Safe Bot library and is prefixed `fsb:`.
 * An XML snippet shows an example offset below. The angular offset is an axis-angle
 * representation of a rotation in radians.
 *
 * ```xml
 * <element name="elname">
 *   <fsb:origin_offset xyz="0 0 0" rotvec="0 0 0" />
 * </element>
 * ```
 *
 * @param fname Name of file being parsed
 * @param body_name Parent body element name containing origin offset
 * @param body_xml Parent element containing origin offset
 * @param err Error status returned after parsing
 * @return Linear and angular coordinate transform offset
 */
MotionVector urdf_parse_origin_offset(
    const std::string& fname, const std::string& body_name, const tinyxml2::XMLElement* body_xml, UrdfError& err);

/**
 * @}
 */

} // namespace fsb
