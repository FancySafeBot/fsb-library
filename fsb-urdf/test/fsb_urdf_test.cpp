
#include <doctest/doctest.h>
#include "fsb_urdf_name_map.h"
#include "fsb_urdf.h"
#include "fsb_body_tree.h"
#include "fsb_test_macros.h"

#include <iostream>

TEST_SUITE_BEGIN("urdf_parse");

TEST_CASE("Parse URDF file" * doctest::description("[urdf_parse][fsb::parse_urdf_file]"))
{
    // Inputs
    const std::string fname = "data/ur5/ur5.urdf";
    // Expected
    const size_t num_bodies = 11U;
    const size_t num_joints = 10U;
    fsb::urdf::UrdfNameMap urdf_names_expected = {};
    for (size_t k = 0U; k < num_bodies; ++k)
    {
        fsb::urdf::NameMapError err = urdf_names_expected.add_body(k, "body" + std::to_string(k));
        REQUIRE(err == fsb::urdf::NameMapError::SUCCESS);
    }
    for (size_t k = 0U; k < num_joints; ++k)
    {
        fsb::urdf::NameMapError err = urdf_names_expected.add_joint(k, "joint" + std::to_string(static_cast<int>(k)));
        REQUIRE(err == fsb::urdf::NameMapError::SUCCESS);
    }
    // const fsb::BodyTree model_expected = {};
    // Process
    fsb::urdf::UrdfError err = {};
    fsb::urdf::UrdfNameMap urdf_names_actual = {};
    const fsb::BodyTree model_actual = fsb::urdf::parse_urdf_file(fname.c_str(), urdf_names_actual, err);
    (void)model_actual;
    std::cout << err.get_description() << "\n";
    REQUIRE(!err.is_error());
    REQUIRE(err.get_description().empty());

    REQUIRE(urdf_names_actual.get_num_bodies() == urdf_names_expected.get_num_bodies());
    REQUIRE(urdf_names_actual.get_num_joints() == urdf_names_expected.get_num_joints());
}

TEST_CASE("Parse URDF ur5 joint limits" * doctest::description("[urdf_parse][fsb::parse_urdf_file]"))
{
    // Inputs
    const std::string fname = "data/ur5/ur5.urdf";
    // Expected
    const std::string joint_name = "shoulder_pan_joint"; // Joint name to test limits
    const fsb::real_t expected_lower_position = -6.28318530718;
    const fsb::real_t expected_upper_position = 6.28318530718;
    const fsb::real_t expected_max_velocity = 3.15;
    const bool expected_set = true;
    // Process
    fsb::urdf::UrdfError err = {};
    fsb::urdf::UrdfNameMap urdf_names_actual = {};
    const fsb::BodyTree model_actual = fsb::urdf::parse_urdf_file(fname.c_str(), urdf_names_actual, err);

    // Get parsed limit information
    fsb::urdf::NameMapError name_err = {};
    const size_t joint_index = urdf_names_actual.get_joint_index(joint_name, name_err);
    REQUIRE(name_err == fsb::urdf::NameMapError::SUCCESS);

    bool is_set = false;
    fsb::real_t min_position = 0.0;
    fsb::real_t max_position = 0.0;
    fsb::real_t max_velocity = 0.0;
    fsb::BodyTreeError limit_err = model_actual.get_joint_position_limit(joint_index, is_set, min_position, max_position);
    REQUIRE(limit_err == fsb::BodyTreeError::SUCCESS);
    limit_err = model_actual.get_joint_velocity_limit(joint_index, max_velocity);
    REQUIRE(limit_err == fsb::BodyTreeError::SUCCESS);

    REQUIRE(is_set == expected_set);
    REQUIRE(min_position == doctest::Approx(expected_lower_position));
    REQUIRE(max_position == doctest::Approx(expected_upper_position));
    REQUIRE(max_velocity == doctest::Approx(expected_max_velocity));
}

TEST_SUITE_END();
