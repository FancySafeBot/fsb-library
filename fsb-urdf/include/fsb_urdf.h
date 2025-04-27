
#pragma once

#include <string>

#include "fsb_urdf_name_map.h"
#include "fsb_urdf_error.h"
#include "fsb_body_tree.h"

namespace fsb::urdf
{

/**
 * @defgroup UrdfTopic Urdf Parsing
 * @brief Parsing URDF files to get rigid body tree object.
 *
 * @{
 */

/**
 * @brief Parse URDF file string to get body tree object.
 *
 * @param urdf_string String of URDF contents
 * @param name_map Output name map for rigid body tree
 * @param err Error object
 * @return Body tree parsed from URDF
 */
BodyTree parse_urdf_string(const std::string& urdf_string, UrdfNameMap& name_map, UrdfError& err);

/**
 * @brief Parse URDF file to get body tree object.
 *
 * @param urdf_filename Input URDF filename
 * @param name_map Output name map for rigid body tree
 * @param err Error object
 * @return Body tree parsed from URDF
 */
BodyTree parse_urdf_file(const std::string& urdf_filename, UrdfNameMap& name_map, UrdfError& err);

/**
 * @}
 */

} // namespace fsb::urdf
