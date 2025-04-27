
#include <doctest/doctest.h>
#include "fsb_urdf_body.h"
#include "fsb_test_macros.h"

TEST_SUITE_BEGIN("urdf_body");

TEST_CASE("Parse empty body" * doctest::description("[urdf_body][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";

    const char* xml_string = "<?xml version='1.0' encoding='UTF-8'?>"
                             "<link name=\"body_name\"></link>";

    const fsb::urdf::UrdfLink body_expected = {};
    const bool          expected_err = true;
    const auto          expected_err_type = fsb::urdf::UrdfErrorType::MISSING_BODY_INERTIAL;
    const std::string   expected_err_description
        = "Missing <inertial> element for body 'body_name' in URDF file: file_name";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError    xml_err = doc.Parse(xml_string);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("link");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError      err = {};
    const fsb::urdf::UrdfLink body_actual = fsb::urdf::urdf_parse_link(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(body_expected.mass_props.com.x == body_actual.mass_props.com.x);
    REQUIRE(body_expected.mass_props.com.y == body_actual.mass_props.com.y);
    REQUIRE(body_expected.mass_props.com.z == body_actual.mass_props.com.z);
}

TEST_CASE("Parse body" * doctest::description("[urdf_body][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";

    const char* xml_string = "<?xml version='1.0' encoding='UTF-8'?>"
                             "<link name=\"body_name\">"
                             "<inertial>"
                             "<origin xyz=\"1.0 2.0 3.0\" rpy=\"0.10 -0.5 0.121\" />"
                             "<inertia ixx=\"1.0\" iyy=\"2.0\" izz=\"3.0\" ixy=\"0.1\" ixz=\"0.2\" iyz=\"0.3\" />"
                             "<mass value=\"1.1\" />"
                             "<fsb:origin_offset xyz=\"1.0 2.0 3.0\" rotvec=\"0.1 0.2 0.3\" />"
                             "</inertial>"
                             "</link>";

    fsb::urdf::UrdfLink body_expected = {};
    body_expected.mass_props.com = {1.0, 2.0, 3.0};
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError    xml_err = doc.Parse(xml_string);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("link");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError      err = {};
    const fsb::urdf::UrdfLink body_actual = fsb::urdf::urdf_parse_link(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(body_expected.mass_props.com.x == body_actual.mass_props.com.x);
    REQUIRE(body_expected.mass_props.com.y == body_actual.mass_props.com.y);
    REQUIRE(body_expected.mass_props.com.z == body_actual.mass_props.com.z);
}

TEST_SUITE_END();
