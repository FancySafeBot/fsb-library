
#include "fsb_urdf_error.h"
#include "fsb_urdf_utilities.h"
#include "fsb_urdf_origin.h"
#include "fsb_types.h"
#include "fsb_motion.h"
#include "fsb_rotation.h"
#include <string>
#include <tinyxml2.h>

namespace fsb::urdf
{

Transform urdf_parse_origin(
    const std::string& fname, const std::string& el_name, const tinyxml2::XMLElement* el_xml,
    UrdfError& err)
{
    Transform el_transf = transform_identity();
    if (const tinyxml2::XMLElement* origin_xml = el_xml->FirstChildElement("origin");
        origin_xml != nullptr)
    {
        if (const char* xyz = origin_xml->Attribute("xyz"); xyz != nullptr)
        {
            const auto xyz_str = std::string(xyz);
            const Vec3 xyz_value = string_to_vector(xyz_str, err);
            if (err.is_error())
            {
                err = {err.get_type(),
                       "Invalid origin translation xyz='" + xyz_str + "' for element '" + el_name
                           + "' in URDF file '" + fname + "'"};
            }
            else
            {
                el_transf.translation = xyz_value;
            }
        }

        if (const char* rpy = origin_xml->Attribute("rpy"); (!err.is_error()) && (rpy != nullptr))
        {
            const auto rpy_str = std::string(rpy);
            const Vec3 rpy_value = string_to_vector(rpy_str, err);
            if (err.is_error())
            {
                err = {err.get_type(),
                       "Invalid origin orientation rpy='" + rpy_str + "' for element '" + el_name
                           + "' in URDF file '" + fname + "'"};
            }
            else
            {
                el_transf.rotation = ezyx_to_quat({rpy_value.z, rpy_value.y, rpy_value.x});
                if (el_transf.rotation.qw < 0.0)
                {
                    el_transf.rotation.qw = -el_transf.rotation.qw;
                    el_transf.rotation.qx = -el_transf.rotation.qx;
                    el_transf.rotation.qy = -el_transf.rotation.qy;
                    el_transf.rotation.qz = -el_transf.rotation.qz;
                }
            }
        }
        else if (const char* quat = origin_xml->Attribute("quat");
                 (!err.is_error()) && (quat != nullptr))
        {
            const auto       quat_str = std::string(quat);
            const Quaternion quat_value = string_to_quaternion(quat_str, err);
            if (err.is_error())
            {
                err= {err.get_type(),
                       "Invalid origin orientation quat='" + quat_str + "' for element '" + el_name
                           + "' in URDF file '" + fname + "'"};
            }
            else
            {
                el_transf.rotation = quat_value;
            }
        }
    }
    return el_transf;
}

MotionVector urdf_parse_origin_offset(
    const std::string& fname, const std::string& body_name, const tinyxml2::XMLElement* body_xml,
    UrdfError& err)
{
    MotionVector origin_offset = {};
    if (const tinyxml2::XMLElement* origin_xml = body_xml->FirstChildElement("fsb:origin_offset");
        origin_xml != nullptr)
    {
        if (const char* xyz = origin_xml->Attribute("xyz"); xyz != nullptr)
        {
            const auto xyz_str = std::string(xyz);
            const Vec3 xyz_value = string_to_vector(xyz_str, err);
            if (err.is_error())
            {
                err = {err.get_type(),
                       "Invalid offset translation xyz='" + xyz_str + "' for body '" + body_name
                           + "' in URDF file '" + fname + "'"};
            }
            else
            {
                origin_offset.linear = xyz_value;
            }
        }
        if (const char* rotvec = origin_xml->Attribute("rotvec");
            (!err.is_error()) && (rotvec != nullptr))
        {
            const auto rotvec_str = std::string(rotvec);
            const Vec3 rotvec_value = string_to_vector(rotvec_str, err);
            if (err.is_error())
            {
                err = {err.get_type(),
                       "Invalid offset rotation rotvec='" + rotvec_str + "' for body '" + body_name
                           + "' in URDF file '" + fname + "'"};
            }
            else
            {
                origin_offset.angular = rotvec_value;
            }
        }
    }
    return origin_offset;
}

} // namespace fsb::urdf
