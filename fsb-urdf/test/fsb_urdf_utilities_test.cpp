
#include <doctest/doctest.h>
#include "fsb_urdf_utilities.h"

TEST_SUITE_BEGIN("urdf_string_conversions");

TEST_CASE("Convert string to Quaternion" * doctest::description("[fsb_string_conversions][fsb::urdf::string_to_quaternion]"))
{
    // Inputs
    const std::string str = " -0.174963553055941  0.933138949631687 -0.116642368703961 -0.291605921759902";
    // Expected
    const fsb::Quaternion q_expected = {0.174963553055941, -0.933138949631687, 0.116642368703961,
                                        0.291605921759902};
    // Process
    fsb::urdf::UrdfError err_actual = {};
    fsb::Quaternion q_actual = fsb::urdf::string_to_quaternion(str, err_actual);

    REQUIRE(!err_actual.is_error());
    REQUIRE(err_actual.get_description().empty());

    REQUIRE(fsb::quat_norm(q_actual) == 1.0);
    REQUIRE(q_actual.qw == q_expected.qw);
    REQUIRE(q_actual.qx == q_expected.qx);
    REQUIRE(q_actual.qy == q_expected.qy);
    REQUIRE(q_actual.qz == q_expected.qz);
}

TEST_CASE("Convert string to vector" * doctest::description("[fsb_string_conversions][fsb::urdf::string_to_vector]"))
{
    // Inputs
    const std::string str = "0.174963553055941 -0.933138949631687  0.116642368703961 ";
    // Expected
    const fsb::Vec3 v_expected = {0.174963553055941, -0.933138949631687, 0.116642368703961};
    // Process
    fsb::urdf::UrdfError err_actual = {};
    fsb::Vec3 v_actual = fsb::urdf::string_to_vector(str, err_actual);

    REQUIRE(!err_actual.is_error());
    REQUIRE(err_actual.get_description().empty());

    REQUIRE(v_actual.x == v_expected.x);
    REQUIRE(v_actual.y == v_expected.y);
    REQUIRE(v_actual.z == v_expected.z);
}

TEST_CASE("Convert string to index" * doctest::description("[fsb_string_conversions][fsb::urdf::string_to_index]"))
{
    // Inputs
    const std::string str = "18723 ";
    // Expected
    const size_t val_expected = 18723U;
    // Process
    fsb::urdf::UrdfError err_actual = {};
    size_t val_actual = fsb::urdf::string_to_index(str, err_actual);

    REQUIRE(!err_actual.is_error());
    REQUIRE(err_actual.get_description().empty());
    REQUIRE(val_actual == val_expected);
}

TEST_CASE("Convert string to real value" * doctest::description("[fsb_string_conversions][fsb::urdf::string_to_real]"))
{
    // Inputs
    const std::string str = "-1e-15 ";
    // Expected
    const fsb::real_t val_expected = -1.0e-15;
    // Process
    fsb::urdf::UrdfError err_actual = {};
    fsb::real_t val_actual = fsb::urdf::string_to_real(str, err_actual);

    REQUIRE(!err_actual.is_error());
    REQUIRE(err_actual.get_description().empty());
    REQUIRE(val_actual == val_expected);
}

TEST_SUITE_END();
