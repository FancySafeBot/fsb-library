
#include <doctest/doctest.h>
#include "fsb_urdf_origin.h"
#include "fsb_test_macros.h"

TEST_SUITE_BEGIN("urdf_origin");

TEST_CASE("Parse identity origin XML" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><origin xyz=\"0.0 0.0 0.0\" rpy=\"0.0 0.0 0.0\"/></root>";
    const fsb::Transform tr_expected = fsb::transform_identity();
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::Transform tr_actual = fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(tr_actual.translation.x == tr_expected.translation.x);
    REQUIRE(tr_actual.translation.y == tr_expected.translation.y);
    REQUIRE(tr_actual.translation.z == tr_expected.translation.z);

    REQUIRE(tr_actual.rotation.qw == tr_expected.rotation.qw);
    REQUIRE(tr_actual.rotation.qx == tr_expected.rotation.qx);
    REQUIRE(tr_actual.rotation.qy == tr_expected.rotation.qy);
    REQUIRE(tr_actual.rotation.qz == tr_expected.rotation.qz);
}

TEST_CASE("Parse arbitrary origin XML" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><origin xyz=\"1.0 2.0 3.0\" rpy=\"0.10 -0.5 0.121\"/></root>";

    const fsb::Quaternion quat_expected = { 0.9651834299409254, 0.06327695587919153, -0.24371474027338194, 0.07085265552970922};
    const fsb::Vec3 transl_expected = {1.0, 2.0, 3.0};
    const fsb::Transform tr_expected = {
        quat_expected,
        transl_expected
    };
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::Transform tr_actual = fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(tr_actual.translation.x == tr_expected.translation.x);
    REQUIRE(tr_actual.translation.y == tr_expected.translation.y);
    REQUIRE(tr_actual.translation.z == tr_expected.translation.z);

    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));
}

TEST_CASE("Parse origin XML with quaternion" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><origin xyz=\"1.0 2.0 3.0\" quat=\"0.10 -0.5 0.121 0.0\"/></root>";

    const fsb::Quaternion quat_expected = { 0.19081711005660068, -0.9540855502830033, 0.2308887031684868, 0.0};
    const fsb::Vec3 transl_expected = {1.0, 2.0, 3.0};
    const fsb::Transform tr_expected = {
        quat_expected,
        transl_expected
    };
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::Transform tr_actual = fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(tr_actual.translation.x == tr_expected.translation.x);
    REQUIRE(tr_actual.translation.y == tr_expected.translation.y);
    REQUIRE(tr_actual.translation.z == tr_expected.translation.z);

    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));
}

TEST_CASE("Invalid quaternion" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><origin xyz=\"1.0 2.0 3.0\" quat=\"0.10 -0.5 0.121\"/></root>";
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::VALUE_CONVERSION_FAILED;
    const std::string expected_err_description = "Invalid origin orientation quat='0.10 -0.5 0.121' for element 'element_name' in URDF file 'file_name'";
    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());
}

TEST_CASE("Parse origin XML xyz failure" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><origin xyz=\"inf 2.0 3.0\" rpy=\"0.10 -0.5 0.121\"/></root>";

    const fsb::Transform tr_expected = fsb::transform_identity();
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::RANGE_ERROR;
    const std::string expected_err_description = "Invalid origin translation xyz='inf 2.0 3.0' for element 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::Transform tr_actual = fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(tr_actual.translation.x == tr_expected.translation.x);
    REQUIRE(tr_actual.translation.y == tr_expected.translation.y);
    REQUIRE(tr_actual.translation.z == tr_expected.translation.z);

    REQUIRE(tr_actual.rotation.qw == tr_expected.rotation.qw);
    REQUIRE(tr_actual.rotation.qx == tr_expected.rotation.qx);
    REQUIRE(tr_actual.rotation.qy == tr_expected.rotation.qy);
    REQUIRE(tr_actual.rotation.qz == tr_expected.rotation.qz);
}

TEST_CASE("Parse origin XML rpy failure" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><origin xyz=\"1.0 2.0 3.0\" rpy=\"0.10 -0.5 !0.121\"/></root>";

    const fsb::Transform tr_expected = {
        fsb::quat_identity(), {1.0, 2.0, 3.0}
    };
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::VALUE_CONVERSION_FAILED;
    const std::string expected_err_description = "Invalid origin orientation rpy='0.10 -0.5 !0.121' for element 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::Transform tr_actual = fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());

    REQUIRE(tr_actual.translation.x == tr_expected.translation.x);
    REQUIRE(tr_actual.translation.y == tr_expected.translation.y);
    REQUIRE(tr_actual.translation.z == tr_expected.translation.z);

    REQUIRE(tr_actual.rotation.qw == tr_expected.rotation.qw);
    REQUIRE(tr_actual.rotation.qx == tr_expected.rotation.qx);
    REQUIRE(tr_actual.rotation.qy == tr_expected.rotation.qy);
    REQUIRE(tr_actual.rotation.qz == tr_expected.rotation.qz);

}

TEST_CASE("Parse origin missing element" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root></root>";
    const fsb::Transform tr_expected = fsb::transform_identity();
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::Transform tr_actual = fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(tr_actual.translation.x == tr_expected.translation.x);
    REQUIRE(tr_actual.translation.y == tr_expected.translation.y);
    REQUIRE(tr_actual.translation.z == tr_expected.translation.z);

    REQUIRE(tr_actual.rotation.qw == tr_expected.rotation.qw);
    REQUIRE(tr_actual.rotation.qx == tr_expected.rotation.qx);
    REQUIRE(tr_actual.rotation.qy == tr_expected.rotation.qy);
    REQUIRE(tr_actual.rotation.qz == tr_expected.rotation.qz);

}

TEST_CASE("Parse origin missing rpy attribute" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><origin xyz=\"1.0 2.0 3.0\" /></root>";
    const fsb::Transform tr_expected = {
        fsb::quat_identity(),
        {1.0, 2.0, 3.0}
    };
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::Transform tr_actual = fsb::urdf::urdf_parse_origin(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(tr_actual.translation.x == tr_expected.translation.x);
    REQUIRE(tr_actual.translation.y == tr_expected.translation.y);
    REQUIRE(tr_actual.translation.z == tr_expected.translation.z);

    REQUIRE(tr_actual.rotation.qw == tr_expected.rotation.qw);
    REQUIRE(tr_actual.rotation.qx == tr_expected.rotation.qx);
    REQUIRE(tr_actual.rotation.qy == tr_expected.rotation.qy);
    REQUIRE(tr_actual.rotation.qz == tr_expected.rotation.qz);

}

TEST_CASE("Parse zero origin offset XML" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><fsb:origin_offset xyz=\"0.0 0.0 0.0\" rotvec=\"0.0 0.0 0.0\"/></root>";
    const fsb::MotionVector offset_expected = {};
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::MotionVector offset_actual = fsb::urdf::urdf_parse_origin_offset(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(offset_actual.angular.x == offset_expected.angular.x);
    REQUIRE(offset_actual.angular.y == offset_expected.angular.y);
    REQUIRE(offset_actual.angular.z == offset_expected.angular.z);

    REQUIRE(offset_actual.linear.x == offset_expected.linear.x);
    REQUIRE(offset_actual.linear.y == offset_expected.linear.y);
    REQUIRE(offset_actual.linear.z == offset_expected.linear.z);
}

TEST_CASE("Parse arbitrary origin offset XML" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";

    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><fsb:origin_offset xyz=\"1.0 2.0 3.0\" rotvec=\"0.1 0.2 0.3\"/></root>";

    const fsb::MotionVector offset_expected = {
        {0.1, 0.2, 0.3},
        {1.0, 2.0, 3.0}
    };
    const bool expected_err = false;

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    const fsb::MotionVector offset_actual = fsb::urdf::urdf_parse_origin_offset(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());

    REQUIRE(offset_actual.angular.x == offset_expected.angular.x);
    REQUIRE(offset_actual.angular.y == offset_expected.angular.y);
    REQUIRE(offset_actual.angular.z == offset_expected.angular.z);

    REQUIRE(offset_actual.linear.x == offset_expected.linear.x);
    REQUIRE(offset_actual.linear.y == offset_expected.linear.y);
    REQUIRE(offset_actual.linear.z == offset_expected.linear.z);
}

TEST_CASE("Invalid origin offset translation" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><fsb:origin_offset xyz=\"0.0 0.0 z\" rotvec=\"0.0 0.0 0.0\"/></root>";
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::VALUE_CONVERSION_FAILED;
    const std::string expected_err_description = "Invalid offset translation xyz='0.0 0.0 z' for body 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::urdf::urdf_parse_origin_offset(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());
}

TEST_CASE("Invalid origin offset rotation" * doctest::description("[urdf_origin][fsb::urdf::urdf_parse_origin]"))
{
    const std::string file_name = "file_name";
    const std::string element_name = "element_name";
    const char *xml_string_input =
        "<?xml version='1.0' encoding='UTF-8'?>"
        "<root><fsb:origin_offset xyz=\"0.0 0.0 0.0\" rotvec=\"0.0 0.0\"/></root>";
    const bool expected_err = true;
    const auto expected_err_type = fsb::urdf::UrdfErrorType::VALUE_CONVERSION_FAILED;
    const std::string expected_err_description = "Invalid offset rotation rotvec='0.0 0.0' for body 'element_name' in URDF file 'file_name'";

    // parse xml
    tinyxml2::XMLDocument doc = {};
    tinyxml2::XMLError xml_err = doc.Parse(xml_string_input);
    REQUIRE(xml_err == tinyxml2::XMLError::XML_SUCCESS);
    const tinyxml2::XMLElement* xml = doc.RootElement();
    REQUIRE(xml != nullptr);

    // process
    fsb::urdf::UrdfError err = {};
    fsb::urdf::urdf_parse_origin_offset(file_name, element_name, xml, err);

    REQUIRE(expected_err == err.is_error());
    REQUIRE(expected_err_type == err.get_type());
    REQUIRE(expected_err_description == err.get_description());
}

TEST_SUITE_END();
