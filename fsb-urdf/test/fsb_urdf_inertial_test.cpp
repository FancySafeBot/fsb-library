
#include <doctest/doctest.h>
#include "fsb_urdf_inertial.h"
#include "fsb_test_macros.h"

TEST_SUITE_BEGIN("urdf_inertial");


TEST_CASE("Parse inertial missing mass" * doctest::description("[urdf_inertial][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<inertial><inertia /></inertial>";
    const fsb::Inertia inertia_expected = {};
    const fsb::real_t mass_expected = 0.0;
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::MISSING_BODY_MASS;
    const std::string expected_err_description = "Missing <mass> element for body 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("inertial");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::real_t mass_actual = 0.0;
    const fsb::Inertia inertia_actual = fsb::urdf::urdf_parse_inertia_mass(file_name, element_name, xml, mass_actual, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(mass_actual == mass_expected);
    REQUIRE(inertia_actual.ixx == inertia_expected.ixx);
    REQUIRE(inertia_actual.iyy == inertia_expected.iyy);
    REQUIRE(inertia_actual.izz == inertia_expected.izz);
    REQUIRE(inertia_actual.ixy == inertia_expected.ixy);
    REQUIRE(inertia_actual.ixz == inertia_expected.ixz);
    REQUIRE(inertia_actual.iyz == inertia_expected.iyz);
}

TEST_CASE("Parse inertial missing mass attribute" * doctest::description("[urdf_inertial][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<inertial><inertia ixx=\"1.0\" iyy=\"2.0\" izz=\"3.0\" /><mass /></inertial>";
    const fsb::Inertia inertia_expected = {};
    const fsb::real_t mass_expected = 0.0;
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::MISSING_MASS_VALUE_ATTRIBUTE;
    const std::string expected_err_description = "Missing mass value attribute for body 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("inertial");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::real_t mass_actual = 0.0;
    const fsb::Inertia inertia_actual = fsb::urdf::urdf_parse_inertia_mass(file_name, element_name, xml, mass_actual, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(mass_actual == mass_expected);
    REQUIRE(inertia_actual.ixx == inertia_expected.ixx);
    REQUIRE(inertia_actual.iyy == inertia_expected.iyy);
    REQUIRE(inertia_actual.izz == inertia_expected.izz);
    REQUIRE(inertia_actual.ixy == inertia_expected.ixy);
    REQUIRE(inertia_actual.ixz == inertia_expected.ixz);
    REQUIRE(inertia_actual.iyz == inertia_expected.iyz);
}

TEST_CASE("Parse inertial missing inertia" * doctest::description("[urdf_inertial][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<inertial><mass /></inertial>";
    const fsb::Inertia inertia_expected = {};
    const fsb::real_t mass_expected = 0.0;
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::MISSING_BODY_INERTIA;
    const std::string expected_err_description = "Missing <inertia> element for body 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("inertial");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::real_t mass_actual = 0.0;
    const fsb::Inertia inertia_actual = fsb::urdf::urdf_parse_inertia_mass(file_name, element_name, xml, mass_actual, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(mass_actual == mass_expected);
    REQUIRE(inertia_actual.ixx == inertia_expected.ixx);
    REQUIRE(inertia_actual.iyy == inertia_expected.iyy);
    REQUIRE(inertia_actual.izz == inertia_expected.izz);
    REQUIRE(inertia_actual.ixy == inertia_expected.ixy);
    REQUIRE(inertia_actual.ixz == inertia_expected.ixz);
    REQUIRE(inertia_actual.iyz == inertia_expected.iyz);
}

TEST_CASE("Parse inertial missing inertia attribute" * doctest::description("[urdf_inertial][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<inertial><inertia ixx=\"1.0\" izz=\"3.0\" /><mass value=\"1.0\" /></inertial>";
    const fsb::Inertia inertia_expected = {};
    const fsb::real_t mass_expected = 1.0;
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::MISSING_INERTIA_ATTRIBUTE;
    const std::string expected_err_description = "Missing inertia attribute iyy for body 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("inertial");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::real_t mass_actual = 0.0;
    const fsb::Inertia inertia_actual = fsb::urdf::urdf_parse_inertia_mass(file_name, element_name, xml, mass_actual, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(mass_actual == mass_expected);
    REQUIRE(inertia_actual.ixx == inertia_expected.ixx);
    REQUIRE(inertia_actual.iyy == inertia_expected.iyy);
    REQUIRE(inertia_actual.izz == inertia_expected.izz);
    REQUIRE(inertia_actual.ixy == inertia_expected.ixy);
    REQUIRE(inertia_actual.ixz == inertia_expected.ixz);
    REQUIRE(inertia_actual.iyz == inertia_expected.iyz);
}

TEST_CASE("Parse inertial principal only" * doctest::description("[urdf_inertial][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<inertial><inertia ixx=\"1.0\" iyy=\"2.0\" izz=\"3.0\" /><mass value=\"1.1\" /></inertial>";
    const fsb::Inertia inertia_expected = {1.0, 2.0, 3.0, 0.0, 0.0, 0.0};
    const fsb::real_t mass_expected = 1.1;
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("inertial");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::real_t mass_actual = 0.0;
    const fsb::Inertia inertia_actual = fsb::urdf::urdf_parse_inertia_mass(file_name, element_name, xml, mass_actual, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(mass_actual == mass_expected);
    REQUIRE(inertia_actual.ixx == inertia_expected.ixx);
    REQUIRE(inertia_actual.iyy == inertia_expected.iyy);
    REQUIRE(inertia_actual.izz == inertia_expected.izz);
    REQUIRE(inertia_actual.ixy == inertia_expected.ixy);
    REQUIRE(inertia_actual.ixz == inertia_expected.ixz);
    REQUIRE(inertia_actual.iyz == inertia_expected.iyz);
}

TEST_CASE("Parse inertial not positive definite" * doctest::description("[urdf_inertial][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<inertial><inertia ixx=\"1.0\" iyy=\"2.0\" izz=\"3.0\" ixy=\"4.0\" ixz=\"5.0\" iyz=\"6.0\" /><mass value=\"1.1\" /></inertial>";
    const fsb::Inertia inertia_expected = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
    const fsb::real_t mass_expected = 1.1;
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::BODY_INERTIA_NOT_POSITIVE_DEFINITE;
    const std::string expected_err_description = "Inertia is not positive definite for body 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("inertial");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::real_t mass_actual = 0.0;
    const fsb::Inertia inertia_actual = fsb::urdf::urdf_parse_inertia_mass(file_name, element_name, xml, mass_actual, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(mass_actual == mass_expected);
    REQUIRE(inertia_actual.ixx == inertia_expected.ixx);
    REQUIRE(inertia_actual.iyy == inertia_expected.iyy);
    REQUIRE(inertia_actual.izz == inertia_expected.izz);
    REQUIRE(inertia_actual.ixy == inertia_expected.ixy);
    REQUIRE(inertia_actual.ixz == inertia_expected.ixz);
    REQUIRE(inertia_actual.iyz == inertia_expected.iyz);
}

TEST_CASE("Parse inertial" * doctest::description("[urdf_inertial][fsb::urdf::urdf_parse_inertia_mass]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<inertial><inertia ixx=\"1.0\" iyy=\"2.0\" izz=\"3.0\" ixy=\"0.1\" ixz=\"0.2\" iyz=\"0.3\" /><mass value=\"1.1\" /></inertial>";
    const fsb::Inertia inertia_expected = {1.0, 2.0, 3.0, 0.1, 0.2, 0.3};
    const fsb::real_t mass_expected = 1.1;
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("inertial");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::real_t mass_actual = 0.0;
    const fsb::Inertia inertia_actual = fsb::urdf::urdf_parse_inertia_mass(file_name, element_name, xml, mass_actual, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(mass_actual == mass_expected);
    REQUIRE(inertia_actual.ixx == inertia_expected.ixx);
    REQUIRE(inertia_actual.iyy == inertia_expected.iyy);
    REQUIRE(inertia_actual.izz == inertia_expected.izz);
    REQUIRE(inertia_actual.ixy == inertia_expected.ixy);
    REQUIRE(inertia_actual.ixz == inertia_expected.ixz);
    REQUIRE(inertia_actual.iyz == inertia_expected.iyz);
}

TEST_SUITE_END();
