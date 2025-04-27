
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_body.h"
#include "fsb_rotation.h"

TEST_SUITE_BEGIN("body");

TEST_CASE("Rotate inertia" * doctest::description("[fsb_body][fsb::body_rotate_inertia]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.461283965309215, 0.607100248856612, 0.205771002489209, -0.613436782171915};
    const fsb::Inertia inertia = {5745.0, 7073.0, 2614.0, -55.722221, 136.4, 49.635477};
    // Expected
    const fsb::Inertia inertia_expected = {5579.999188067047, 4088.6616643639954, 5763.339147568986, -1937.2422433060353, 756.0449295800091, 312.29076917576685};
    // Process
    fsb::Inertia inertia_actual = fsb::body_rotate_inertia(q_a, inertia);

    REQUIRE(inertia_actual.ixx == FsbApprox(inertia_expected.ixx));
    REQUIRE(inertia_actual.iyy == FsbApprox(inertia_expected.iyy));
    REQUIRE(inertia_actual.izz == FsbApprox(inertia_expected.izz));
    REQUIRE(inertia_actual.ixy == FsbApprox(inertia_expected.ixy));
    REQUIRE(inertia_actual.ixz == FsbApprox(inertia_expected.ixz));
    REQUIRE(inertia_actual.iyz == FsbApprox(inertia_expected.iyz));
}

TEST_CASE("Inertia parallel axis" * doctest::description("[fsb_body][fsb::body_parallel_axis_inertia]"))
{
    // Inputs
    const fsb::real_t mass = 0.845456;
    const fsb::Vec3 com = {-0.509, 13.117, 6.045};
    const fsb::Inertia inertia = {5745.0, 7073.0, 2614.0, -55.722221, 136.4, 49.635477};
    // Expected
    const fsb::Inertia inertia_expected = {5921.360188887584, 7104.113715874336, 2759.68455618512, -50.077489206832, 139.00138779368, -17.402644197839997};
    // Process
    fsb::Inertia inertia_actual = fsb::body_parallel_axis_inertia(mass, com, inertia);

    REQUIRE(inertia_actual.ixx == FsbApprox(inertia_expected.ixx));
    REQUIRE(inertia_actual.iyy == FsbApprox(inertia_expected.iyy));
    REQUIRE(inertia_actual.izz == FsbApprox(inertia_expected.izz));
    REQUIRE(inertia_actual.ixy == FsbApprox(inertia_expected.ixy));
    REQUIRE(inertia_actual.ixz == FsbApprox(inertia_expected.ixz));
    REQUIRE(inertia_actual.iyz == FsbApprox(inertia_expected.iyz));
}

TEST_CASE("Transform mass properties coordinates" * doctest::description("[fsb_body][fsb::body_transform_mass_props]"))
{
    // constexpr fsb::real_t inertia_epsilon = 1e-7;
    // Inputs
    const fsb::Transform tr_a = {
         {0.461283965309215, 0.607100248856612, 0.205771002489209, -0.613436782171915},
        {1.02, -0.998, 0.223}
    };
    const fsb::MassProps mass_props = {
        0.845456,
        {-0.509, 13.117, 6.045},
        {5745.0, 7073.0, 2614.0, -55.722221, 136.4, 49.635477}
    };
    // Expected
    const fsb::MassProps mass_props_expected = {
        0.845456,
        {8.282865430212299, -12.173013376125553, 5.811081673172163},
       {5579.999188067047, 4088.6616643639954, 5763.339147568986, -1937.2422433060353, 756.0449295800091, 312.29076917576685}
    };
    // Process
    fsb::MassProps mass_props_actual = fsb::body_transform_mass_props(tr_a, mass_props);

    // Check result
    REQUIRE(mass_props_actual.mass == FsbApprox(mass_props_expected.mass));
    REQUIRE(mass_props_actual.com.x == FsbApprox(mass_props_expected.com.x));
    REQUIRE(mass_props_actual.com.y == FsbApprox(mass_props_expected.com.y));
    REQUIRE(mass_props_actual.com.z == FsbApprox(mass_props_expected.com.z));
    REQUIRE(mass_props_actual.inertia.ixx == FsbApprox(mass_props_expected.inertia.ixx));
    REQUIRE(mass_props_actual.inertia.iyy == FsbApprox(mass_props_expected.inertia.iyy));
    REQUIRE(mass_props_actual.inertia.izz == FsbApprox(mass_props_expected.inertia.izz));
    REQUIRE(mass_props_actual.inertia.ixy == FsbApprox(mass_props_expected.inertia.ixy));
    REQUIRE(mass_props_actual.inertia.ixz == FsbApprox(mass_props_expected.inertia.ixz));
    REQUIRE(mass_props_actual.inertia.iyz == FsbApprox(mass_props_expected.inertia.iyz));
}

TEST_CASE("Combine mass properties" * doctest::description("[fsb_body][fsb::body_combine]"))
{
    // Inputs
    const fsb::MassProps mass_props_a = {
        0.845456,
        {-0.509, 13.117, 6.045},
        {5745.0, 7073.0, 2614.0, -55.722, 136.4, 49.635}
    };
    const fsb::Transform tr_ab = {
        {0.461283965309215, 0.607100248856612, 0.205771002489209, -0.613436782171915},
        {1.02, -0.998, 0.223}
    };
    const fsb::MassProps mass_props_b = {
        1.6553,
         {0.123, 1.23, -0.98},
        {2362, 3652, 867, -25.722, 100.4, 55.23}
    };
    // Expected
    const fsb::MassProps mass_props_expected = {
        2.500756,
        {1.5405191422974733, 3.876614168373854, 2.250091862735077},
         {8557.488932209628, 8868.694110458548, 5152.456628555167, -1229.6903263521663, 689.8302218241186, 69.624330704779}
    };
    // Process
    fsb::MassProps mass_props_actual = fsb::body_combine(mass_props_a, tr_ab, mass_props_b);

    REQUIRE(mass_props_actual.mass == FsbApprox(mass_props_expected.mass));
    REQUIRE(mass_props_actual.com.x == FsbApprox(mass_props_expected.com.x));
    REQUIRE(mass_props_actual.com.y == FsbApprox(mass_props_expected.com.y));
    REQUIRE(mass_props_actual.com.z == FsbApprox(mass_props_expected.com.z));
    REQUIRE(mass_props_actual.inertia.ixx == FsbApprox(mass_props_expected.inertia.ixx));
    REQUIRE(mass_props_actual.inertia.iyy == FsbApprox(mass_props_expected.inertia.iyy));
    REQUIRE(mass_props_actual.inertia.izz == FsbApprox(mass_props_expected.inertia.izz));
    REQUIRE(mass_props_actual.inertia.ixy == FsbApprox(mass_props_expected.inertia.ixy));
    REQUIRE(mass_props_actual.inertia.ixz == FsbApprox(mass_props_expected.inertia.ixz));
    REQUIRE(mass_props_actual.inertia.iyz == FsbApprox(mass_props_expected.inertia.iyz));
}

TEST_CASE("Principal axis orientation of inertia" * doctest::description("[fsb_body][fsb::body_inertia_principal_axis]"))
{
    // Inputs
    const fsb::Inertia inertia = {2715.90886514019, 2294.094219069441, 4494.196915790364, -772.7895675672958, -91.87901808579488, 694.326268037918};
    // Expected
    const fsb::PrincipalInertia principal_inertia_expected = {
        {0.20577100248920885, -0.6134367821719154, -0.4612839653092141, -0.6071002488566114},
        {4745.0, 3145.2, 1614.0, 0.0, 0.0, 0.0}
    };
    const bool retval_expected = true;
    // Process
    fsb::PrincipalInertia principal_inertia_actual = {};
    const bool retval_actual = fsb::body_inertia_principal_axis(inertia, principal_inertia_actual);
    // recover the original inertia tensor
    const fsb::Inertia inertia_orig_actual = fsb::body_rotate_inertia(principal_inertia_actual.rot, principal_inertia_actual.inertia);

    REQUIRE(retval_actual == retval_expected);

    REQUIRE(principal_inertia_actual.inertia.ixx == FsbApprox(principal_inertia_expected.inertia.ixx));
    REQUIRE(principal_inertia_actual.inertia.iyy == FsbApprox(principal_inertia_expected.inertia.iyy));
    REQUIRE(principal_inertia_actual.inertia.izz == FsbApprox(principal_inertia_expected.inertia.izz));
    REQUIRE(principal_inertia_actual.inertia.ixy == FsbApprox(principal_inertia_expected.inertia.ixy));
    REQUIRE(principal_inertia_actual.inertia.ixz == FsbApprox(principal_inertia_expected.inertia.ixz));
    REQUIRE(principal_inertia_actual.inertia.iyz == FsbApprox(principal_inertia_expected.inertia.iyz));

    REQUIRE(principal_inertia_actual.rot.qw == FsbApprox(principal_inertia_expected.rot.qw));
    REQUIRE(principal_inertia_actual.rot.qx == FsbApprox(principal_inertia_expected.rot.qx));
    REQUIRE(principal_inertia_actual.rot.qy == FsbApprox(principal_inertia_expected.rot.qy));
    REQUIRE(principal_inertia_actual.rot.qz == FsbApprox(principal_inertia_expected.rot.qz));

    REQUIRE(inertia_orig_actual.ixx == FsbApprox(inertia.ixx));
    REQUIRE(inertia_orig_actual.iyy == FsbApprox(inertia.iyy));
    REQUIRE(inertia_orig_actual.izz == FsbApprox(inertia.izz));
    REQUIRE(inertia_orig_actual.ixy == FsbApprox(inertia.ixy));
    REQUIRE(inertia_orig_actual.ixz == FsbApprox(inertia.ixz));
    REQUIRE(inertia_orig_actual.iyz == FsbApprox(inertia.iyz));
}

TEST_CASE("Validate Inertia is not positive definite" * doctest::description("[fsb_body][fsb::body_validate_inertia_is_pd]"))
{
    // Inputs
    const fsb::Inertia inertia = {0.814723686393179, 0.632359246225410, 0.957506835434298, 0.905791937075619, 0.126986816293506, 0.0975404049994095};
    // Expected
    const bool retval_expected = false;
    // Process
    const bool retval_actual = fsb::body_validate_inertia_is_pd(inertia);
    REQUIRE(retval_expected == retval_actual);
}

TEST_CASE("Validate Inertia is positive definite" * doctest::description("[fsb_body][fsb::body_validate_inertia_is_pd]"))
{
    // Inputs
    const fsb::Inertia inertia = {1.814723686393179, 1.632359246225410, 1.957506835434298, 0.905791937075619, 0.126986816293506, 0.0975404049994095};
    // Expected
    const bool retval_expected = true;
    // Process
    const bool retval_actual = fsb::body_validate_inertia_is_pd(inertia);
    REQUIRE(retval_expected == retval_actual);
}

TEST_SUITE_END();
