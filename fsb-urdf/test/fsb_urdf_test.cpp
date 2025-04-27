
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

    // REQUIRE(model_actual.sizes.num_bodies == model_expected.sizes.num_bodies);
    // REQUIRE(model_actual.sizes.num_joints == model_expected.sizes.num_joints);
    // REQUIRE(model_actual.sizes.num_coordinates == model_expected.sizes.num_coordinates);
    // REQUIRE(model_actual.sizes.num_dofs == model_expected.sizes.num_dofs);
    // REQUIRE(model_actual.base_body_index == model_expected.base_body_index);
}

TEST_SUITE_END();
