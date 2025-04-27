
#include <cmath>
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_rotation.h"

TEST_SUITE_BEGIN("rotation");

TEST_CASE("Skew symmetric matrix from vector" * doctest::description("[fsb_rotation][fsb::skew_symmetric]"))
{
    // Inputs
    const fsb::Vec3 vec = {0.12, -0.23, 0.983};
    // Expected
    const fsb::Mat3 skew_expected = {0.0, 0.983, 0.23, -0.983, 0.0, 0.12, -0.23, -0.12, 0.0};
    // Process
    const fsb::Mat3 skew_actual = fsb::skew_symmetric(vec);

    REQUIRE(skew_actual.m00 == FsbApprox(skew_expected.m00));
    REQUIRE(skew_actual.m10 == FsbApprox(skew_expected.m10));
    REQUIRE(skew_actual.m20 == FsbApprox(skew_expected.m20));
    REQUIRE(skew_actual.m01 == FsbApprox(skew_expected.m01));
    REQUIRE(skew_actual.m11 == FsbApprox(skew_expected.m11));
    REQUIRE(skew_actual.m21 == FsbApprox(skew_expected.m21));
    REQUIRE(skew_actual.m02 == FsbApprox(skew_expected.m02));
    REQUIRE(skew_actual.m12 == FsbApprox(skew_expected.m12));
    REQUIRE(skew_actual.m22 == FsbApprox(skew_expected.m22));
}

TEST_CASE("Identity rotation matrix" * doctest::description("[fsb_rotation][fsb::identity]"))
{
    // Expected
    const fsb::Mat3 identity_expected = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    // Process
    const fsb::Mat3 identity_actual = fsb::rot_identity();

    REQUIRE(identity_actual.m00 == FsbApprox(identity_expected.m00));
    REQUIRE(identity_actual.m10 == FsbApprox(identity_expected.m10));
    REQUIRE(identity_actual.m20 == FsbApprox(identity_expected.m20));
    REQUIRE(identity_actual.m01 == FsbApprox(identity_expected.m01));
    REQUIRE(identity_actual.m11 == FsbApprox(identity_expected.m11));
    REQUIRE(identity_actual.m21 == FsbApprox(identity_expected.m21));
    REQUIRE(identity_actual.m02 == FsbApprox(identity_expected.m02));
    REQUIRE(identity_actual.m12 == FsbApprox(identity_expected.m12));
    REQUIRE(identity_actual.m22 == FsbApprox(identity_expected.m22));
}

TEST_CASE("Rotation matrix about x-axis" * doctest::description("[fsb_rotation][fsb::rot_rx]"))
{
    // Inputs
    const fsb::real_t rx = 0.23;
    // Expected
    const fsb::Mat3 rot_expected = {1.0,
                                    0.0,
                                    0.0,
                                    0.0,
                                    0.9736663950053749,
                                    0.2279775235351884,
                                    0.0,
                                    -0.2279775235351884,
                                    0.9736663950053749};
    // Process
    const fsb::Mat3 rot_actual = fsb::rot_rx(rx);

    REQUIRE(rot_actual.m00 == FsbApprox(rot_expected.m00));
    REQUIRE(rot_actual.m10 == FsbApprox(rot_expected.m10));
    REQUIRE(rot_actual.m20 == FsbApprox(rot_expected.m20));
    REQUIRE(rot_actual.m01 == FsbApprox(rot_expected.m01));
    REQUIRE(rot_actual.m11 == FsbApprox(rot_expected.m11));
    REQUIRE(rot_actual.m21 == FsbApprox(rot_expected.m21));
    REQUIRE(rot_actual.m02 == FsbApprox(rot_expected.m02));
    REQUIRE(rot_actual.m12 == FsbApprox(rot_expected.m12));
    REQUIRE(rot_actual.m22 == FsbApprox(rot_expected.m22));
}

TEST_CASE("Rotation matrix about y-axis" * doctest::description("[fsb_rotation][fsb::rot_ry]"))
{
    // Inputs
    const fsb::real_t ry = -0.14;
    // Expected
    const fsb::Mat3 rot_expected = {0.9902159962126371,  0.0, 0.1395431146442365, 0.0, 1.0, 0.0,
                                    -0.1395431146442365, 0.0, 0.9902159962126371};
    // Process
    const fsb::Mat3 rot_actual = fsb::rot_ry(ry);

    REQUIRE(rot_actual.m00 == FsbApprox(rot_expected.m00));
    REQUIRE(rot_actual.m10 == FsbApprox(rot_expected.m10));
    REQUIRE(rot_actual.m20 == FsbApprox(rot_expected.m20));
    REQUIRE(rot_actual.m01 == FsbApprox(rot_expected.m01));
    REQUIRE(rot_actual.m11 == FsbApprox(rot_expected.m11));
    REQUIRE(rot_actual.m21 == FsbApprox(rot_expected.m21));
    REQUIRE(rot_actual.m02 == FsbApprox(rot_expected.m02));
    REQUIRE(rot_actual.m12 == FsbApprox(rot_expected.m12));
    REQUIRE(rot_actual.m22 == FsbApprox(rot_expected.m22));
}

TEST_CASE("Rotation matrix about z-axis" * doctest::description("[fsb_rotation][fsb::rot_rz]"))
{
    // Inputs
    const fsb::real_t rz = 0.5;
    // Expected
    const fsb::Mat3 rot_expected = {0.8775825618903728,
                                    0.479425538604203,
                                    0.0,
                                    -0.479425538604203,
                                    0.8775825618903728,
                                    0.0,
                                    0.0,
                                    0.0,
                                    1.0};
    // Process
    const fsb::Mat3 rot_actual = fsb::rot_rz(rz);

    REQUIRE(rot_actual.m00 == FsbApprox(rot_expected.m00));
    REQUIRE(rot_actual.m10 == FsbApprox(rot_expected.m10));
    REQUIRE(rot_actual.m20 == FsbApprox(rot_expected.m20));
    REQUIRE(rot_actual.m01 == FsbApprox(rot_expected.m01));
    REQUIRE(rot_actual.m11 == FsbApprox(rot_expected.m11));
    REQUIRE(rot_actual.m21 == FsbApprox(rot_expected.m21));
    REQUIRE(rot_actual.m02 == FsbApprox(rot_expected.m02));
    REQUIRE(rot_actual.m12 == FsbApprox(rot_expected.m12));
    REQUIRE(rot_actual.m22 == FsbApprox(rot_expected.m22));
}

TEST_CASE("Convert 90 deg y-rotation quaternion to Euler angles mobile ZYX" * doctest::description("[fsb_rotation][fsb::quat_to_ezyx]"))
{
    // Inputs
    const fsb::Quaternion q_a = {1.0 / sqrt(2.0), 0.0, 1.0 / sqrt(2.0), 0.0};
    // Expected
    const fsb::Vec3 e_expected = {0.0, M_PI_2, 0.0};
    // Process
    fsb::Vec3 e_actual = fsb::quat_to_ezyx(q_a);

    REQUIRE(e_expected.x == FsbApprox(e_actual.x));
    REQUIRE(e_expected.y == FsbApprox(e_actual.y));
    REQUIRE(e_expected.z == FsbApprox(e_actual.z));
}

TEST_CASE("Convert -90 deg y-rotation quaternion to Euler angles mobile ZYX" * doctest::description("[fsb_rotation][fsb::quat_to_ezyx]"))
{
    // Inputs
    const fsb::Quaternion q_a = {1.0 / sqrt(2.0), 0.0, -1.0 / sqrt(2.0), 0.0};
    // Expected
    const fsb::Vec3 e_expected = {0.0, -M_PI_2, 0.0};
    // Process
    fsb::Vec3 e_actual = fsb::quat_to_ezyx(q_a);

    REQUIRE(e_expected.x == FsbApprox(e_actual.x));
    REQUIRE(e_expected.y == FsbApprox(e_actual.y));
    REQUIRE(e_expected.z == FsbApprox(e_actual.z));
}

TEST_CASE("Convert quaternion to Euler angles mobile ZYX" * doctest::description("[fsb_rotation][fsb::quat_to_ezyx]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                 0.424264068711929};
    // Expected
    const fsb::Vec3 e_expected = {2.93460045936997,	-0.613826938436634,	-2.14848442100611};
    // Process
    fsb::Vec3 e_actual = fsb::quat_to_ezyx(q_a);

    REQUIRE(e_expected.x == FsbApprox(e_actual.x));
    REQUIRE(e_expected.y == FsbApprox(e_actual.y));
    REQUIRE(e_expected.z == FsbApprox(e_actual.z));
}

TEST_CASE("Convert Euler angles mobile ZYX to quaternion" * doctest::description("[fsb_rotation][fsb::ezyx_to_quat]"))
{
    // Inputs
    const fsb::Vec3 e_a = {0.14, -0.986, 1.01};
    // Expected
    const fsb::Quaternion q_expected = {0.753053064372948, 0.454120397679548, -0.383370894562464, 0.282334028859893};
    // Process
    fsb::Quaternion q_actual = fsb::ezyx_to_quat(e_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Convert quaternion to rotation matrix" * doctest::description("[fsb_rotation][fsb::quat_to_rot]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                 0.424264068711929};
    // Expected
    const fsb::Mat3 r_expected = {-0.8000000000000007, 0.16800000000000034, 0.5760000000000002,
                                  -0.3600000000000003, 0.6335999999999992,  -0.6848000000000009,
                                  -0.4800000000000001, -0.7552000000000009, -0.4463999999999999};
    // Process
    fsb::Mat3 r_actual = fsb::quat_to_rot(q_a);

    REQUIRE(r_actual.m00 == FsbApprox(r_expected.m00));
    REQUIRE(r_actual.m10 == FsbApprox(r_expected.m10));
    REQUIRE(r_actual.m20 == FsbApprox(r_expected.m20));
    REQUIRE(r_actual.m01 == FsbApprox(r_expected.m01));
    REQUIRE(r_actual.m11 == FsbApprox(r_expected.m11));
    REQUIRE(r_actual.m21 == FsbApprox(r_expected.m21));
    REQUIRE(r_actual.m02 == FsbApprox(r_expected.m02));
    REQUIRE(r_actual.m12 == FsbApprox(r_expected.m12));
    REQUIRE(r_actual.m22 == FsbApprox(r_expected.m22));
}

TEST_CASE("Convert rotation matrix to quaternion" * doctest::description("[fsb_rotation][fsb_rot_to_quat]"))
{
    // Inputs
    const fsb::Mat3 r_a = {-0.8000000000000007, 0.16800000000000034, 0.5760000000000002,
                           -0.3600000000000003, 0.6335999999999992,  -0.6848000000000009,
                           -0.4800000000000001, -0.7552000000000009, -0.4463999999999999};
    // Expected
    const fsb::Quaternion q_expected = {0.311126983722081, 0.0565685424949238, -0.848528137423857,
                                        0.424264068711929};
    // Process
    fsb::Quaternion q_actual = fsb::rot_to_quat(r_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Convert rotation matrix (tr < 0) to quaternion" * doctest::description("[fsb_rotation][fsb_rot_to_quat]"))
{
    // Inputs
    const fsb::Mat3 r_a = {-0.972871299079089, -0.0705752490039160, -0.220319244861181,
                           0.216339880812362, 0.0598777445071503, -0.974480226419618,
                           0.0819664040827632, -0.995707682977676, -0.0429850981267873};;
    // Expected
    const fsb::Quaternion q_expected = {0.104906324048260, 0.0505866942499405, 0.720370415431017, -0.683741262548405};
    // Process
    fsb::Quaternion q_actual = fsb::rot_to_quat(r_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Convert identity rotation matrix to quaternion" * doctest::description("[fsb_rotation][fsb_rot_to_quat]"))
{
    // Inputs
    const fsb::Mat3 r_a = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0};
    // Expected
    const fsb::Quaternion q_expected = {1.0, 0.0, 0.0, 0.0};
    // Process
    fsb::Quaternion q_actual = fsb::rot_to_quat(r_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Convert 180 deg x-axis rotation matrix to quaternion" * doctest::description("[fsb_rotation][fsb_rot_to_quat]"))
{
    // Inputs
    const fsb::Mat3 r_a = {
        1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, -1.0};
    // Expected
    const fsb::Quaternion q_expected = {0.0, 1.0, 0.0, 0.0};
    // Process
    fsb::Quaternion q_actual = fsb::rot_to_quat(r_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Convert 180 deg z-axis rotation matrix to quaternion" * doctest::description("[fsb_rotation][fsb_rot_to_quat]"))
{
    // Inputs
    const fsb::Mat3 r_a = {
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0,
        0.0, 0.0, 1.0};
    // Expected
    const fsb::Quaternion q_expected = {0.0, 0.0, 0.0, 1.0};
    // Process
    fsb::Quaternion q_actual = fsb::rot_to_quat(r_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Convert quaternion to rotation vector" * doctest::description("[fsb_rotation][fsb::quat_to_rotvec]"))
{
    // Inputs
    const fsb::Quaternion q_a = {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929};
    // Expected
    const fsb::Vec3 rtovec_expected = { 0.14933279325714682, -2.2399918988572023, 1.1199959494286023};
    // Process
    fsb::Vec3 e_actual = fsb::quat_to_rotvec(q_a);

    REQUIRE(rtovec_expected.x == FsbApprox(e_actual.x));
    REQUIRE(rtovec_expected.y == FsbApprox(e_actual.y));
    REQUIRE(rtovec_expected.z == FsbApprox(e_actual.z));
}

TEST_CASE("Convert identity quaternion to zero rotation vector" * doctest::description("[fsb_rotation][fsb::quat_to_rotvec]"))
{
    // Inputs
    const fsb::Quaternion q_a = {1.0, 0.9 * FSB_TOL, 0.0, 0.0};
    // Expected
    const fsb::Vec3 rtovec_expected = { 0.0, 0.0, 0.0};
    // Process
    fsb::Vec3 e_actual = fsb::quat_to_rotvec(q_a);

    REQUIRE(rtovec_expected.x == FsbApprox(e_actual.x));
    REQUIRE(rtovec_expected.y == FsbApprox(e_actual.y));
    REQUIRE(rtovec_expected.z == FsbApprox(e_actual.z));
}

TEST_CASE("Convert rotation vector to quaternion" * doctest::description("[fsb_rotation][fsb::rotvec_to_quat]"))
{
    // Inputs
    const fsb::Vec3 rotvec_a = {0.075, -0.12, 0.56};
    // Expected
    const fsb::Quaternion q_expected = {0.958585928771424, 0.0369808805636469, -0.059169408901835044, 0.27612390820856364};
    // Process
    fsb::Quaternion q_actual = fsb::rotvec_to_quat(rotvec_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_CASE("Convert zero rotation vector to identity quaternion" * doctest::description("[fsb_rotation][fsb::rotvec_to_quat]"))
{
    // Inputs
    const fsb::Vec3 rotvec_a = {0.9 * FSB_TOL, 0.0, 0.0};
    // Expected
    const fsb::Quaternion q_expected = {1.0, 0.0, 0.0, 0.0};
    // Process
    fsb::Quaternion q_actual = fsb::rotvec_to_quat(rotvec_a);

    REQUIRE(fsb::quat_norm(q_actual) == FsbApprox(1.0));
    REQUIRE(q_actual.qw == FsbApprox(q_expected.qw));
    REQUIRE(q_actual.qx == FsbApprox(q_expected.qx));
    REQUIRE(q_actual.qy == FsbApprox(q_expected.qy));
    REQUIRE(q_actual.qz == FsbApprox(q_expected.qz));
}

TEST_SUITE_END();
