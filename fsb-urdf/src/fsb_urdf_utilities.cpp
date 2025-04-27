
#include <cmath>
#include <limits>
#include <cstdlib>
#include <string>
#include <vector>

#include "fsb_quaternion.h"
#include "fsb_types.h"
#include "fsb_urdf_error.h"
#include "fsb_urdf_utilities.h"

#include "fsb_configuration.h"

namespace fsb::urdf
{

size_t string_to_index(const std::string& str, UrdfError& err)
{
    const char*   start = str.c_str();
    char*         end = nullptr;
    constexpr int base = 10;
    long          value = std::strtol(start, &end, base);

    if (end == str)
    {
        err = {UrdfErrorType::VALUE_CONVERSION_FAILED, "'" + str + "' is not an integer"};
        value = 0;
    }
    else if (value < 0 || (static_cast<size_t>(value) >= MaxSize::index))
    {
        const auto str_len = static_cast<size_t>(end - start);
        err
            = {UrdfErrorType::RANGE_ERROR,
               "Value (" + str.substr(str_len) + ") is larger than max array length ("
                   + std::to_string(MaxSize::index) + ")"};
        value = 0;
    }
    else
    {
        // no error
    }
    return static_cast<size_t>(value);
}

real_t string_to_real(const std::string& str, UrdfError& err)
{
    const char* start = str.c_str();
    char*       end = nullptr;
    double_t    value = std::strtod(start, &end);

    if (end == str)
    {
        err = {UrdfErrorType::VALUE_CONVERSION_FAILED, "'" + str + "' is not a number"};
        value = 0.0;
    }
    else if (
        std::isinf(value) || (value >= std::numeric_limits<real_t>::max())
        || (value <= std::numeric_limits<real_t>::lowest()))
    {
        const auto str_len = static_cast<size_t>(end - start);
        err
            = {UrdfErrorType::RANGE_ERROR,
               "'" + str.substr(str_len) + "' value is out of range of type `real_t`"};
        value = 0.0;
    }
    else
    {
        // no error
    }
    return static_cast<real_t>(value);
}

std::vector<std::string> split_string_spaces(const std::string& str)
{
    const std::string        space = " ";
    std::vector<std::string> result = {};
    size_t                   start = 0U;
    size_t                   end = str.find_first_of(space, start);
    while (end != std::string::npos)
    {
        if (const size_t str_len = end - start; str_len > 0U)
        {
            result.push_back(str.substr(start, str_len));
        }
        start = end + 1U;
        end = str.find_first_of(space, start);
    }
    if (start < str.length())
    {
        result.push_back(str.substr(start));
    }
    return result;
}

Vec3 string_to_vector(const std::string& str, UrdfError& err)
{
    Vec3                           result = {};
    const std::vector<std::string> str_array = split_string_spaces(str);
    if (str_array.size() != 3U)
    {
        err
            = {UrdfErrorType::VALUE_CONVERSION_FAILED,
               "Conversion failed to Vec3 due to string '" + str
                   + "' not split into 3 substrings separated by spaces."};
    }

    if (!err.is_error())
    {
        result.x = string_to_real(str_array.at(0U), err);
    }
    if (!err.is_error())
    {
        result.y = string_to_real(str_array.at(1U), err);
    }
    if (!err.is_error())
    {
        result.z = string_to_real(str_array.at(2U), err);
    }

    return result;
}

Quaternion string_to_quaternion(const std::string& str, UrdfError& err)
{
    Quaternion                     result = {};
    const std::vector<std::string> str_array = split_string_spaces(str);
    if (str_array.size() != 4U)
    {
        err
            = {UrdfErrorType::VALUE_CONVERSION_FAILED,
               "Conversion failed to Quaternion due to string '" + str
                   + "' not split into 4 substrings separated by spaces."};
    }

    if (!err.is_error())
    {
        result.qw = string_to_real(str_array.at(0U), err);
    }
    if (!err.is_error())
    {
        result.qx = string_to_real(str_array.at(1U), err);
    }
    if (!err.is_error())
    {
        result.qy = string_to_real(str_array.at(2U), err);
    }
    if (!err.is_error())
    {
        result.qz = string_to_real(str_array.at(3U), err);
    }
    quat_normalize(result);

    return result;
}

} // namespace fsb::urdf
