
#include <doctest/doctest.h>
#include "fsb_urdf_joint.h"
#include "fsb_test_macros.h"

TEST_SUITE_BEGIN("urdf_joint");

TEST_CASE("Parse empty joint" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"continuous\"></joint>";

    const fsb::urdf::UrdfJoint joint_expected = {};
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::MISSING_JOINT_PARENT;
    const std::string expected_err_description = "Joint 'joint_name' <parent> element is missing in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(joint_expected.parent_child_transform.translation.x == joint_actual.parent_child_transform.translation.x);
    REQUIRE(joint_expected.parent_child_transform.translation.y == joint_actual.parent_child_transform.translation.y);
    REQUIRE(joint_expected.parent_child_transform.translation.z == joint_actual.parent_child_transform.translation.z);
}

TEST_CASE("Parse joint limit, only velocity" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"revolute\">"
            "<limit velocity=\"100.0\" />"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
        "</joint>";

    fsb::urdf::UrdfJoint joint_expected = {};
    joint_expected.parent_name = "parent_name";
    joint_expected.child_name = "child_name";
    joint_expected.limits.lower_position = 0.0;
    joint_expected.limits.upper_position = 0.0;
    joint_expected.limits.max_velocity = 100.0;
    joint_expected.limits.set = false;
    joint_expected.parent_child_transform = fsb::transform_identity();
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(joint_expected.parent_name == joint_actual.parent_name);
    REQUIRE(joint_expected.child_name == joint_actual.child_name);

    REQUIRE(joint_expected.parent_child_transform.translation.x == joint_actual.parent_child_transform.translation.x);
    REQUIRE(joint_expected.parent_child_transform.translation.y == joint_actual.parent_child_transform.translation.y);
    REQUIRE(joint_expected.parent_child_transform.translation.z == joint_actual.parent_child_transform.translation.z);

    REQUIRE(joint_expected.limits.lower_position == joint_actual.limits.lower_position);
    REQUIRE(joint_expected.limits.upper_position == joint_actual.limits.upper_position);
    REQUIRE(joint_expected.limits.max_velocity == joint_actual.limits.max_velocity);
    REQUIRE(joint_expected.limits.set == joint_actual.limits.set);

    REQUIRE(joint_expected.parent_child_transform.rotation.qw == FsbApprox(joint_actual.parent_child_transform.rotation.qw));
    REQUIRE(joint_expected.parent_child_transform.rotation.qx == FsbApprox(joint_actual.parent_child_transform.rotation.qx));
    REQUIRE(joint_expected.parent_child_transform.rotation.qy == FsbApprox(joint_actual.parent_child_transform.rotation.qy));
    REQUIRE(joint_expected.parent_child_transform.rotation.qz == FsbApprox(joint_actual.parent_child_transform.rotation.qz));
}

TEST_CASE("Parse joint" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"continuous\">"
            "<limit lower=\"-0.125\" upper=\"1.3e5\" velocity=\"100.0\" />"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
            "<origin xyz=\"1.0 2.0 3.0\" rpy=\"0.10 -0.5 0.121\" />"
            "<axis xyz=\"0 0 -1\" />"
        "</joint>";

    fsb::urdf::UrdfJoint joint_expected = {};
    joint_expected.parent_name = "parent_name";
    joint_expected.child_name = "child_name";
    joint_expected.limits.lower_position = -0.125;
    joint_expected.limits.upper_position = 1.3e5;
    joint_expected.limits.max_velocity = 100.0;
    joint_expected.limits.set = true;
    joint_expected.reversed = true;
    joint_expected.joint_type = fsb::JointType::REVOLUTE_Z;
    joint_expected.parent_child_transform.rotation = { 0.9651834299409254, 0.06327695587919153, -0.24371474027338194, 0.07085265552970922};
    joint_expected.parent_child_transform.translation = {1.0, 2.0, 3.0};
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(joint_expected.parent_name == joint_actual.parent_name);
    REQUIRE(joint_expected.child_name == joint_actual.child_name);

    REQUIRE(joint_expected.limits.lower_position == joint_actual.limits.lower_position);
    REQUIRE(joint_expected.limits.upper_position == joint_actual.limits.upper_position);
    REQUIRE(joint_expected.limits.max_velocity == joint_actual.limits.max_velocity);
    REQUIRE(joint_expected.limits.set == joint_actual.limits.set);

    REQUIRE(joint_expected.joint_type == joint_actual.joint_type);
    REQUIRE(joint_expected.reversed == joint_actual.reversed);

    REQUIRE(joint_expected.parent_child_transform.translation.x == joint_actual.parent_child_transform.translation.x);
    REQUIRE(joint_expected.parent_child_transform.translation.y == joint_actual.parent_child_transform.translation.y);
    REQUIRE(joint_expected.parent_child_transform.translation.z == joint_actual.parent_child_transform.translation.z);

    REQUIRE(joint_expected.parent_child_transform.rotation.qw == FsbApprox(joint_actual.parent_child_transform.rotation.qw));
    REQUIRE(joint_expected.parent_child_transform.rotation.qx == FsbApprox(joint_actual.parent_child_transform.rotation.qx));
    REQUIRE(joint_expected.parent_child_transform.rotation.qy == FsbApprox(joint_actual.parent_child_transform.rotation.qy));
    REQUIRE(joint_expected.parent_child_transform.rotation.qz == FsbApprox(joint_actual.parent_child_transform.rotation.qz));
}

TEST_CASE("Failed joint parsing with invalid axis direction" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"continuous\">"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
            "<axis xyz=\"0 1 1\" />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::JOINT_INVALID_AXIS;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}

TEST_CASE("Failed joint parsing with non-vector axis" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"prismatic\">"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
            "<axis xyz=\"x\" />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::JOINT_INVALID_AXIS;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}

TEST_CASE("Failed joint parsing with missing name" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint type=\"continuous\">"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::MISSING_NAME;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}

TEST_CASE("Failed joint parsing with missing type" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\">"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::MISSING_JOINT_TYPE;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}

TEST_CASE("Parse Cartesian joint" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"cartesian\">"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
        "</joint>";

    fsb::urdf::UrdfJoint joint_expected = {};
    joint_expected.parent_name = "parent_name";
    joint_expected.child_name = "child_name";
    joint_expected.parent_child_transform = fsb::transform_identity();
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(joint_expected.parent_name == joint_actual.parent_name);
    REQUIRE(joint_expected.child_name == joint_actual.child_name);

    REQUIRE(joint_expected.parent_child_transform.translation.x == joint_actual.parent_child_transform.translation.x);
    REQUIRE(joint_expected.parent_child_transform.translation.y == joint_actual.parent_child_transform.translation.y);
    REQUIRE(joint_expected.parent_child_transform.translation.z == joint_actual.parent_child_transform.translation.z);

    REQUIRE(joint_expected.parent_child_transform.rotation.qw == FsbApprox(joint_actual.parent_child_transform.rotation.qw));
    REQUIRE(joint_expected.parent_child_transform.rotation.qx == FsbApprox(joint_actual.parent_child_transform.rotation.qx));
    REQUIRE(joint_expected.parent_child_transform.rotation.qy == FsbApprox(joint_actual.parent_child_transform.rotation.qy));
    REQUIRE(joint_expected.parent_child_transform.rotation.qz == FsbApprox(joint_actual.parent_child_transform.rotation.qz));
}

TEST_CASE("Parse Spherical joint" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"spherical\">"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
        "</joint>";

    fsb::urdf::UrdfJoint joint_expected = {};
    joint_expected.parent_name = "parent_name";
    joint_expected.child_name = "child_name";
    joint_expected.parent_child_transform = fsb::transform_identity();
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(joint_expected.parent_name == joint_actual.parent_name);
    REQUIRE(joint_expected.child_name == joint_actual.child_name);

    REQUIRE(joint_expected.parent_child_transform.translation.x == joint_actual.parent_child_transform.translation.x);
    REQUIRE(joint_expected.parent_child_transform.translation.y == joint_actual.parent_child_transform.translation.y);
    REQUIRE(joint_expected.parent_child_transform.translation.z == joint_actual.parent_child_transform.translation.z);

    REQUIRE(joint_expected.parent_child_transform.rotation.qw == FsbApprox(joint_actual.parent_child_transform.rotation.qw));
    REQUIRE(joint_expected.parent_child_transform.rotation.qx == FsbApprox(joint_actual.parent_child_transform.rotation.qx));
    REQUIRE(joint_expected.parent_child_transform.rotation.qy == FsbApprox(joint_actual.parent_child_transform.rotation.qy));
    REQUIRE(joint_expected.parent_child_transform.rotation.qz == FsbApprox(joint_actual.parent_child_transform.rotation.qz));
}

TEST_CASE("Failed joint parsing with missing parent link" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"spherical\">"
            "<parent />"
            "<child link=\"child_name\" />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::MISSING_PARENT_LINK_ATTRIBUTE;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}

TEST_CASE("Failed joint parsing with missing child link" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"spherical\">"
            "<parent link=\"parent_name\" />"
            "<child />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::MISSING_CHILD_LINK_ATTRIBUTE;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}

TEST_CASE("Failed joint parsing with missing child element" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"spherical\">"
            "<parent link=\"parent_name\" />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::MISSING_JOINT_CHILD;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}


TEST_CASE("Failed joint parsing with invalid joint type" * doctest::description("[urdf_joint][fsb::urdf_parse_joint]"))
{
    const std::string file_name = "file_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<joint name=\"joint_name\" type=\"not a type\">"
            "<parent link=\"parent_name\" />"
            "<child link=\"child_name\" />"
        "</joint>";

    const bool expected_err = true;
    const auto expected_error_type = fsb::urdf::UrdfErrorType::INVALID_JOINT_TYPE;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.FirstChildElement("joint");
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::urdf::UrdfJoint joint_actual = fsb::urdf::urdf_parse_joint(file_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_error_type == err.get_type());
}

TEST_SUITE_END();
