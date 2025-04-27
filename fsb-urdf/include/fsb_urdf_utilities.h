
#pragma once

#include <cstddef>
#include <vector>
#include <string>

#include "fsb_types.h"
#include "fsb_quaternion.h"
#include "fsb_urdf_error.h"

namespace fsb::urdf
{

/**
 * @addtogroup UrdfTopic
 * @{
 */

/**
 * @brief Parse a string to extract an unsigned integer as index value.
 *
 * The standard function `std::strtol` is used to parse the string and ignores trailing string
 * characters after parsing the value.
 *
 * @param str String to convert to an index value (unsigned integer)
 * @param err Error from parsing
 * @return Index value
 */
size_t string_to_index(const std::string& str, UrdfError& err);

/**
 * @brief Parse a string to extract a real value.
 *
 * The standard function `std::strtod` is used to parse the string and ignores trailing string
 * characters after parsing the value.
 *
 * @param str String to convert to a real value
 * @param err Error from parsing
 * @return Real value
 */
real_t string_to_real(const std::string& str, UrdfError& err);

/**
 * @brief Split a string with space character as delimiter
 *
 * @param str String to split
 * @return Array of strings between space characters.
 */
std::vector<std::string> split_string_spaces(const std::string& str);

/**
 * @brief Converts a string to a vector with 3 elements separated by the space character.
 *
 * @param str Convert string to 3-element vector
 * @param err Error from parsing string.
 * @return Vector from string
 */
Vec3 string_to_vector(const std::string& str, UrdfError& err);

/**
 * @brief Converts a string to a quaternion.
 *
 * @param str String to convert to a quaternion
 * @param err Error from parsing string.
 * @return Quaternion from string
 */
Quaternion string_to_quaternion(const std::string& str, UrdfError& err);

/**
 * @}
 */

} // namespace fsb
