
#include "fsb_urdf_inertial.h"
#include "fsb_urdf_error.h"
#include "fsb_urdf_utilities.h"

#include "fsb_types.h"
#include "fsb_body.h"
#include "tinyxml2.h"

namespace fsb::urdf
{

static Inertia urdf_parse_inertia_elements(const std::string& fname, const std::string& body_name, const tinyxml2::XMLElement* inertia_xml, UrdfError& err)
{
    Inertia body_inertia = {};

    const char* ixx = inertia_xml->Attribute("ixx");
    const char* iyy = inertia_xml->Attribute("iyy");
    const char* izz = inertia_xml->Attribute("izz");
    const char* ixy = inertia_xml->Attribute("ixy");
    const char* ixz = inertia_xml->Attribute("ixz");
    const char* iyz = inertia_xml->Attribute("iyz");

    if (ixx == nullptr)
    {
        err = {UrdfErrorType::MISSING_INERTIA_ATTRIBUTE, "Missing inertia attribute ixx for body '" + body_name + "' in URDF file '" + fname + "'"};
    }
    else if (iyy == nullptr)
    {
        err = {UrdfErrorType::MISSING_INERTIA_ATTRIBUTE, "Missing inertia attribute iyy for body '" + body_name + "' in URDF file '" + fname + "'"};
    }
    else if (izz == nullptr)
    {
        err = {UrdfErrorType::MISSING_INERTIA_ATTRIBUTE, "Missing inertia attribute izz for body '" + body_name + "' in URDF file '" + fname + "'"};
    }

    if (!err.is_error())
    {
        const auto ixx_str = std::string(ixx);
        body_inertia.ixx = string_to_real(std::string(ixx), err);
        if (err.is_error())
        {
            err = {err.get_type(), "Invalid inertia ixx='" + ixx_str + "' for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }
    if (!err.is_error())
    {
        const auto iyy_str = std::string(iyy);
        body_inertia.iyy = string_to_real(iyy_str, err);
        if (err.is_error())
        {
            err = {err.get_type(), "Invalid inertia iyy='" + iyy_str + "' for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }
    if (!err.is_error())
    {
        const auto izz_str = std::string(izz);
        body_inertia.izz = string_to_real(izz_str, err);
        if (err.is_error())
        {
            err = {err.get_type(), "Invalid inertia izz='" + izz_str + "' for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }
    if (!err.is_error() && (ixy != nullptr))
    {
        const auto ixy_str = std::string(ixy);
        body_inertia.ixy = string_to_real(ixy_str, err);
        if (err.is_error())
        {
            err = {err.get_type(), "Invalid inertia ixy='" + ixy_str + "' for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }
    if (!err.is_error() && (ixz != nullptr))
    {
        const auto ixz_str = std::string(ixz);
        body_inertia.ixz = string_to_real(ixz_str, err);
        if (err.is_error())
        {
            err = {err.get_type(), "Invalid inertia ixz='" + ixz_str + "' for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }
    if (!err.is_error() && (iyz != nullptr))
    {
        const auto iyz_str = std::string(iyz);
        body_inertia.iyz = string_to_real(iyz_str, err);
        if (err.is_error())
        {
            err = {err.get_type(), "Invalid inertia iyz='" + iyz_str + "' for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }

    if (!err.is_error())
    {
        if (!body_validate_inertia_is_pd(body_inertia))
        {
            err = {UrdfErrorType::BODY_INERTIA_NOT_POSITIVE_DEFINITE, "Inertia is not positive definite for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }

    return body_inertia;
}

Inertia urdf_parse_inertia_mass(
    const std::string& fname, const std::string& body_name, const tinyxml2::XMLElement* inertial_xml, real_t& body_mass,
    UrdfError& err)
{
    Inertia body_inertia = {};
    body_mass = 0.0;

    const tinyxml2::XMLElement* mass_xml = inertial_xml->FirstChildElement("mass");
    const tinyxml2::XMLElement* inertia_xml = inertial_xml->FirstChildElement("inertia");
    if (mass_xml == nullptr)
    {
        err = {UrdfErrorType::MISSING_BODY_MASS, "Missing <mass> element for body '" + body_name + "' in URDF file '" + fname + "'"};
    }
    else if (inertia_xml == nullptr)
    {
        err = {UrdfErrorType::MISSING_BODY_INERTIA, "Missing <inertia> element for body '" + body_name + "' in URDF file '" + fname + "'"};
    }

    if (!err.is_error() && (mass_xml != nullptr))
    {
        if (const char* mass = mass_xml->Attribute("value");
            mass != nullptr)
        {
            const auto mass_str = std::string(mass);
            body_mass = string_to_real(mass_str, err);
            if (err.is_error())
            {
                err = {err.get_type(), "Invalid mass value='" + mass_str + "' for body '" + body_name + "' in URDF file '" + fname + "'"};
            }
        }
        else
        {
            err = {UrdfErrorType::MISSING_MASS_VALUE_ATTRIBUTE, "Missing mass value attribute for body '" + body_name + "' in URDF file '" + fname + "'"};
        }
    }

    if (!err.is_error() && (inertia_xml != nullptr))
    {
        body_inertia = urdf_parse_inertia_elements(fname, body_name, inertia_xml, err);
    }

    return body_inertia;
}

} // namespace fsb
