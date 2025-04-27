
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_spatial.h"

TEST_SUITE_BEGIN("spatial");

TEST_CASE("Coordinate transform to spatial RBDA convention" * doctest::description("[fsb_spatial][fsb::transform_to_spatial]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.8658406131180871, -0.2843509043081139, -0.373283005655497, 0.17356380262961224},
        {0.5, 0.1, -0.05}
    };
    // Expected
    const fsb::Transform tr_expected = {
        {0.8658406131180871, -0.2843509043081138, -0.373283005655497, 0.1735638026296122},
        {-0.35443473778148815, -0.0647679027887825, 0.36425421812535064}
    };
    // Process
    const fsb::Transform tr_actual = transform_to_spatial(tr_a);

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE("Coordinate transform inverse to spatial RBDA convention" * doctest::description("[fsb_spatial][fsb::transform_inverse_to_spatial]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.8658406131180871, -0.2843509043081139, -0.373283005655497, 0.17356380262961224},
        {0.5, 0.1, -0.05}
    };
    // Expected
    const fsb::Transform tr_expected = {
        {0.8658406131180871, 0.2843509043081139, 0.373283005655497, -0.17356380262961224},
        {0.5, 0.1, -0.05}
    };
    // Process
    const fsb::Transform tr_actual = transform_inverse_to_spatial(tr_a);

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE("Velocity in space frame converted to body-fixed frame" * doctest::description("[fsb_spatial][fsb::transform_inverse_to_spatial]"))
{
    const fsb::Transform pose = {
        {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075},
        {0.12, -0.34, 0.921}
    };

    const fsb::MotionVector velocity_space = {};

    const fsb::MotionVector vel_body_expected = {};

    const fsb::MotionVector vel_body_actual = fsb::spatial_space_to_body(pose, velocity_space);

    REQUIRE(vel_body_actual.angular.x == FsbApprox(vel_body_expected.angular.x));
    REQUIRE(vel_body_actual.angular.y == FsbApprox(vel_body_expected.angular.y));
    REQUIRE(vel_body_actual.angular.z == FsbApprox(vel_body_expected.angular.z));
    REQUIRE(vel_body_actual.linear.x == FsbApprox(vel_body_expected.linear.x));
    REQUIRE(vel_body_actual.linear.y == FsbApprox(vel_body_expected.linear.y));
    REQUIRE(vel_body_actual.linear.z == FsbApprox(vel_body_expected.linear.z));

}

TEST_CASE("Spatial velocity in body-fixed frame converted to space frame"  * doctest::description("[fsb_spatial][fsb::transform_inverse_to_spatial]"))
{
    const fsb::Transform pose = {
        {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075},
        {0.0, 0.0, 0.0}
    };
    const fsb::MotionVector velocity_body = {
        {0.1298, -0.723, -1.2},
        {1.324, 0.555, -0.112}
    };

    const fsb::MotionVector vel_space_expected = {
        {-0.10897112641181879, -0.3088257021880492, -1.3683307419175372},
        {-0.46021971916667503, 1.3499282187787538, -0.19858654092101438}
    };

    const fsb::MotionVector vel_space_actual = fsb::spatial_body_to_space(pose, velocity_body);

    REQUIRE(vel_space_actual.angular.x == FsbApprox(vel_space_expected.angular.x));
    REQUIRE(vel_space_actual.angular.y == FsbApprox(vel_space_expected.angular.y));
    REQUIRE(vel_space_actual.angular.z == FsbApprox(vel_space_expected.angular.z));
    REQUIRE(vel_space_actual.linear.x == FsbApprox(vel_space_expected.linear.x));
    REQUIRE(vel_space_actual.linear.y == FsbApprox(vel_space_expected.linear.y));
    REQUIRE(vel_space_actual.linear.z == FsbApprox(vel_space_expected.linear.z));

}

TEST_SUITE_END();
