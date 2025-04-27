
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_quaternion.h"

TEST_SUITE_BEGIN("quaternion");

TEST_CASE("Quaternion multiply" * doctest::description("[fsb_quaternion][fsb::quat_multiply]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.218217890235992, 0.436435780471985, 0.872871560943970, 0.0};
    const fsb::Quaternion q_b = {0.267261241912424, 0.534522483824849, 0.0, 0.801783725737273};
    // Expected
    const fsb::Quaternion q_expected = {-0.174963553055941, 0.933138949631687, -0.116642368703961,
                                        -0.291605921759902};
    // Process
    const fsb::Quaternion q_actual = fsb::quat_multiply(q_a, q_b);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion inverse multiply" * doctest::description("[fsb_quaternion][fsb::quat_multiply]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.218217890235992, 0.436435780471985, 0.872871560943970, 0.0};
    const fsb::Quaternion q_b = {0.267261241912424, 0.534522483824849, 0.0, 0.801783725737273};
    // Expected
    const fsb::Quaternion q_expected = {0.291605921759902, -0.699854212223765, 0.116642368703961,
                                        0.641533027871785};
    // Process
    const fsb::Quaternion q_actual = fsb::quat_multiply(quat_conjugate(q_a), q_b);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion multiply inverse" * doctest::description("[fsb_quaternion][fsb::quat_multiply]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.218217890235992, 0.436435780471985, 0.872871560943970, 0.0};
    const fsb::Quaternion q_b = {0.267261241912424, 0.534522483824849, 0.0, 0.801783725737273};
    // Expected
    const fsb::Quaternion q_expected = {0.291605921759902, -0.699854212223765, 0.583211843519804,
                                        0.291605921759902};
    // Process
    const fsb::Quaternion q_actual = fsb::quat_multiply(q_a, quat_conjugate(q_b));

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion rotate vector" * doctest::description("[fsb_quaternion][fsb::quat_rotate_vector]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.218217890235992, 0.436435780471985, 0.872871560943970, 0.0};
    const fsb::Vec3 p_in = {0.1, 5.0, -2.1};
    // Expected
    const fsb::Vec3 p_expected = {2.95714285714286, 3.57142857142857, 2.81428571428571};
    // Process
    fsb::Vec3 p_actual = fsb::quat_rotate_vector(q_a, p_in);

    REQUIRE(p_actual.x == FsbApprox(p_expected.x));
    REQUIRE(p_actual.y == FsbApprox(p_expected.y));
    REQUIRE(p_actual.z == FsbApprox(p_expected.z));
}

TEST_CASE("Quaternion inverse rotate vector" * doctest::description("[fsb_quaternion][fsb::quat_rotate_vector]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.218217890235992, 0.436435780471985, 0.872871560943970, 0.0};
    const fsb::Vec3 p_in = {0.1, 5.0, -2.1};
    // Expected
    const fsb::Vec3 p_expected = {4.55714285714286, 2.77142857142857, 0.985714285714286};
    // Process
    fsb::Vec3 p_actual = fsb::quat_rotate_vector(quat_conjugate(q_a), p_in);

    REQUIRE(p_actual.x == FsbApprox(p_expected.x));
    REQUIRE(p_actual.y == FsbApprox(p_expected.y));
    REQUIRE(p_actual.z == FsbApprox(p_expected.z));
}

TEST_CASE("Quaternion conjugate" * doctest::description("[fsb_quaternion][fsb::quat_conjugate]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                 0.424264068711929};
    // Expected
    const fsb::Quaternion q_expected = {0.311126983722081, -0.0565685424949238, 0.848528137423857,
                                        -0.424264068711929};
    // Process
    fsb::Quaternion q_actual = fsb::quat_conjugate(q_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion norm" * doctest::description("[fsb_quaternion][fsb::quat_norm]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.218217890235992, 0.436435780471985, 0.872871560943970, 0.0};
    // Expected
    const double norm_expected = 1.0;
    // Process
    const double norm_actual = fsb::quat_norm(q_a);

    REQUIRE(norm_actual == FsbApprox(norm_expected));
}

TEST_CASE("Quaternion normalize" * doctest::description("[fsb_quaternion][fsb::quat_normalize]"))
{
    // Inputs
    const fsb::Quaternion q_a = {-0.501901147542783, -1.00380229508557, -2.00760459017113, 0.0};
    // Expected
    const fsb::Quaternion q_expected = {0.218217890235992, 0.436435780471985, 0.872871560943970,
                                        0.0};
    // Process
    fsb::Quaternion q_actual = q_a;
    fsb::quat_normalize(q_actual);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion normalize under tolerance" * doctest::description("[fsb_quaternion][fsb::quat_normalize]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.0, 3e-15, 0.0, 0.0};
    // Expected
    const fsb::Quaternion q_expected = {1.0, 0.0, 0.0, 0.0};
    // Process
    fsb::Quaternion q_actual = q_a;
    fsb::quat_normalize(q_actual);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion normalize above tolerance" * doctest::description("[fsb_quaternion][fsb::quat_normalize]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.0, 5e-14, 0.0, 0.0};
    // Expected
    const fsb::Quaternion q_expected = {0.0, 1.0, 0.0, 0.0};
    // Process
    fsb::Quaternion q_actual = q_a;
    fsb::quat_normalize(q_actual);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion logarithm of identity" * doctest::description("[fsb_quaternion][fsb::quat_log]"))
{
    // Inputs
    const fsb::Quaternion q_a = {1.0, 0.0, 0.0, 0.0};
    // Expected
    const fsb::Vec3 p_expected = {0.0, 0.0, 0.0};
    // Process
    const fsb::Vec3 p_actual = fsb::quat_log(q_a);

    REQUIRE(p_actual.x == FsbApprox(p_expected.x));
    REQUIRE(p_actual.y == FsbApprox(p_expected.y));
    REQUIRE(p_actual.z == FsbApprox(p_expected.z));
}

TEST_CASE("Quaternion logarithm" * doctest::description("[fsb_quaternion][fsb::quat_log]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                 0.424264068711929};
    // Expected
    const fsb::Vec3 p_expected = {0.0746663966285733, -1.11999594942860, 0.559997974714300};
    // Process
    const fsb::Vec3 p_actual = fsb::quat_log(q_a);

    REQUIRE(p_actual.x == FsbApprox(p_expected.x));
    REQUIRE(p_actual.y == FsbApprox(p_expected.y));
    REQUIRE(p_actual.z == FsbApprox(p_expected.z));
}

TEST_CASE("Quaternion exponential" * doctest::description("[fsb_quaternion][fsb::quat_exp]"))
{
    // Inputs
    const fsb::Vec3 p_in = {0.0746663966285733, -1.11999594942860, 0.559997974714300};
    // Expected
    const fsb::Quaternion q_expected = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                        0.424264068711929};
    // Process
    const fsb::Quaternion q_actual = fsb::quat_exp(p_in);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion exponential of zero angle" * doctest::description("[fsb_quaternion][fsb::quat_exp]"))
{
    // Inputs
    const fsb::Vec3 p_in = {0.0, 0.0, 0.0};
    // Expected
    const fsb::Quaternion q_expected = {1.0, 0.0, 0.0, 0.0};
    // Process
    const fsb::Quaternion q_actual = fsb::quat_exp(p_in);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Quaternion add rotation vector in body frame" * doctest::description("[fsb_quaternion][fsb::quat_boxplus]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                 0.424264068711929};
    const fsb::Vec3 phi = {0.075, -0.12, 0.56};
    // Expected
    const fsb::Quaternion q_expected = {0.12879363305026925, -0.14346389269453994, -0.8317266006042587, 0.5206353555281533};
    // Process
    fsb::Quaternion q_actual = fsb::quat_boxplus(q_a, phi);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Get rotation vector between two quaternions in body frame"
          * doctest::description("[fsb_quaternion][fsb::quat_boxminus]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                 0.424264068711929};
    const fsb::Quaternion q_b = {0.12879363305026925, -0.14346389269453994, -0.8317266006042587, 0.5206353555281533};
    // Expected
    const fsb::Vec3 phi_expected = {0.075, -0.12, 0.56};
    // Process
    fsb::Vec3 phi_actual = fsb::quat_boxminus(q_a, q_b);

    REQUIRE(phi_actual.x == FsbApprox(phi_expected.x));
    REQUIRE(phi_actual.y == FsbApprox(phi_expected.y));
    REQUIRE(phi_actual.z == FsbApprox(phi_expected.z));
}

TEST_CASE("Set identity quaternion" * doctest::description("[fsb_quaternion][fsb::quat_identity]"))
{
    // Expected
    const fsb::Quaternion q_expected = {1.0, 0.0, 0.0, 0.0};
    // Process
    fsb::Quaternion q_actual = fsb::quat_identity();

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Set rotation about x-axis" * doctest::description("[fsb_quaternion][fsb::quat_rx]"))
{
    // Inputs
    const fsb::real_t rx = 45.0 * M_PI / 180.0;
    // Expected
    const fsb::Quaternion q_expected = {0.9238795325112867, 0.3826834323650897, 0.0, 0.0};
    // Process
    fsb::Quaternion q_actual = fsb::quat_rx(rx);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Set rotation about y-axis" * doctest::description("[fsb_quaternion][fsb::quat_ry]"))
{
    // Inputs
    const fsb::real_t ry = 30.0 * M_PI / 180.0;
    // Expected
    const fsb::Quaternion q_expected = {0.9659258262890684, 0.0, 0.25881904510252074, 0.0};
    // Process
    fsb::Quaternion q_actual = fsb::quat_ry(ry);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Set rotation about z-axis" * doctest::description("[fsb_quaternion][fsb::quat_rz]"))
{
    // Inputs
    const fsb::real_t rz = 90.0 * M_PI / 180.0;
    // Expected
    const fsb::Quaternion q_expected = {0.7071067811865476, 0.0, 0.0, 0.7071067811865476};
    // Process
    fsb::Quaternion q_actual = fsb::quat_rz(rz);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_SUITE_END();
