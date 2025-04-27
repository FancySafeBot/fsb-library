
#include "fsb_urdf_error.h"
#include "fsb_urdf_body.h"
#include "fsb_urdf_inertial.h"
#include "fsb_urdf_origin.h"

#include "fsb_types.h"
#include "fsb_motion.h"
#include "fsb_body.h"

#include <string>
#include <tinyxml2.h>

namespace fsb::urdf
{

UrdfLink urdf_parse_link(const std::string& fname, const tinyxml2::XMLElement* link_xml, UrdfError& err)
{
    UrdfLink body = {};

    // attributes
    const char* body_name_item = link_xml->Attribute("name");
    // child elements
    const tinyxml2::XMLElement* inertial_xml = link_xml->FirstChildElement("inertial");

    // body name
    if (body_name_item == nullptr)
    {
        err = {UrdfErrorType::MISSING_NAME, "Link name attribute is missing in URDF file '" + fname + "'"};
    }
    else
    {
        body.link_name = std::string(body_name_item);
    }

    // body transform
    Transform body_transf = transform_identity();
    if (!err.is_error())
    {
        if (inertial_xml == nullptr)
        {
            err = {UrdfErrorType::MISSING_BODY_INERTIAL, "Missing <inertial> element for body '" + body.link_name + "' in URDF file: " + fname};
        }
        else
        {
            body_transf = urdf_parse_origin(fname, body.link_name, inertial_xml, err);
        }
    }

    // body mass properties
    if (!err.is_error())
    {
        real_t body_mass = 0.0;
        const Inertia body_inertia = urdf_parse_inertia_mass(fname, body.link_name, inertial_xml, body_mass, err);
        if (!err.is_error())
        {
            // rotate inertia to align with body frame
            body.mass_props.mass = body_mass;
            body.mass_props.com = body_transf.translation;
            body.mass_props.inertia = body_rotate_inertia(body_transf.rotation, body_inertia);
            if (!body_inertia_principal_axis(body.mass_props.inertia, body.principal_inertia))
            {
                err = {UrdfErrorType::BODY_PRINCIPAL_INERTIA, "Failed to determine inertia principal axis for body '" + body.link_name + "' in URDF file: " + fname};
            }
        }
    }

    // body origin offset
    if (!err.is_error())
    {
        body.origin_offset = urdf_parse_origin_offset(fname, body.link_name, link_xml, err);
    }

    return body;
}

}
