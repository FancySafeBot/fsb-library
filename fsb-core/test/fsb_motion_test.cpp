
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_motion.h"

TEST_SUITE_BEGIN("motion");

TEST_CASE("Transform identity" * doctest::description("[fsb_motion][fsb::transform_identity]"))
{
    // Expected
    const fsb::Transform tr_expected = {
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };
    // Process
    const fsb::Transform tr_actual = fsb::transform_identity();

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE("Transform inverse" * doctest::description("[fsb_motion][fsb::transform_inverse]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    // Expected
    const fsb::Transform tr_expected = {
        {0.3111269837220809, -0.05656854249492381, 0.8485281374238572, -0.4242640687119291},
        {-1.6801600000000005, 2.311968000000003, 1.2746239999999998}
    };
    // Process
    const fsb::Transform tr_actual = transform_inverse(tr_a);

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE("Coordinate transform" * doctest::description("[fsb_motion][fsb::coord_transform]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    const fsb::Transform tr_b = {
        {0.1439620627497104, 0.03914447570262396, -0.6145579329747501, 0.7746361607416852},
        {-0.234, 0.9292, 0.34}
    };
    // Expected
    const fsb::Transform tr_expected = {
        {0.8075438511172749, 0.37624310283319573, 0.3405738610214492, -0.300538685167274},
        {-0.21051200000000017, 0.07266111999999886, 2.1971238399999993}
    };
    // Process
    const fsb::Transform tr_actual = fsb::coord_transform(tr_a, tr_b);

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE("Coordinate inverse transform" * doctest::description("[fsb_motion][fsb::coord_transform]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    const fsb::Transform tr_b = {
        {0.1439620627497104, 0.03914447570262396, -0.6145579329747501, 0.7746361607416852},
        {-0.234, 0.9292, 0.34}
    };
    // Expected
    const fsb::Transform tr_expected = {
        {0.8971248158247275, 0.40060090814267507, -0.04183725099637242, 0.18148173917995314},
        {-1.1410144, 2.7521171200000016, 0.5334361599999989}
    };
    // Process
    const fsb::Transform tr_actual = fsb::coord_transform(transform_inverse(tr_a), tr_b);

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE("Coordinate transform inverse" * doctest::description("[fsb_motion][fsb::coord_transform]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    const fsb::Transform tr_b = {
        {0.1439620627497104, 0.03914447570262396, -0.6145579329747501, 0.7746361607416852},
        {-0.234, 0.9292, 0.34}
    };
    // Expected
    const fsb::Transform tr_expected = {
        {0.8971248158247276, 0.3925305509618235, 0.09626213909203274, -0.1783828242025656},
        {-0.06396280125000789, -0.6097953717002031, 2.195367960750141}
    };
    // Process
    const fsb::Transform tr_actual = fsb::coord_transform(tr_a, transform_inverse(tr_b));

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE("Coordinate transform position" * doctest::description("[fsb_motion][fsb::coord_transform_position]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    const fsb::Vec3 p_in = {-0.234, 0.9292, 0.34};
    // Expected
    const fsb::Vec3 p_expected = {-0.21051200000000017, 0.07266111999999886, 2.1971238399999993};
    // Process
    fsb::Vec3 p_actual = fsb::coord_transform_position(tr_a, p_in);

    REQUIRE(p_actual.x == FsbApprox(p_expected.x));
    REQUIRE(p_actual.y == FsbApprox(p_expected.y));
    REQUIRE(p_actual.z == FsbApprox(p_expected.z));
}

TEST_CASE("Coordinate inverse transform position" * doctest::description("[fsb_motion][fsb::coord_transform_position]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    const fsb::Vec3 p_in = {-0.234, 0.9292, 0.34};
    // Expected
    const fsb::Vec3 p_expected = {-1.1410144, 2.7521171200000016, 0.5334361599999989};
    // Process
    fsb::Vec3 p_actual = fsb::coord_transform_position(transform_inverse(tr_a), p_in);

    REQUIRE(p_actual.x == FsbApprox(p_expected.x));
    REQUIRE(p_actual.y == FsbApprox(p_expected.y));
    REQUIRE(p_actual.z == FsbApprox(p_expected.z));
}

TEST_CASE("Motion transform velocity" * doctest::description("[fsb_motion][fsb::motion_transform_velocity]"))
{
    const fsb::Vec3 parent_position = {0.12, -0.34, 0.921};
    const fsb::Quaternion parent_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
    const fsb::Vec3 parent_vel_ang = {0.123, -0.2, 0.2432};
    const fsb::Vec3 parent_vel_lin = {-0.9, 0.62, 0.89};

    const fsb::Vec3 motion_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion motion_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::Vec3 motion_vel_ang = {0.1298, -0.723, -1.2};
    const fsb::Vec3 motion_vel_lin = {1.324, 0.555, -0.112};

    const fsb::Transform tr_parent = {
        parent_rotation,
        parent_position
    };
    const fsb::MotionVector vel_parent = {
        parent_vel_ang,
        parent_vel_lin
    };
    const fsb::Transform tr_parent_child = {
        motion_rotation,
        motion_position
    };
    const fsb::MotionVector vel_motion = {
        motion_vel_ang,
        motion_vel_lin
    };

    // Expected
    const fsb::Vec3 expected_vel_ang = {0.014028873588181936, -0.5088257021880498, -1.1251307419175365};
    const fsb::Vec3 expected_vel_lin = {-1.4082001593914384, 1.600427631351611, 0.4118142677229711};const fsb::MotionVector vel_expected = {
        expected_vel_ang,
        expected_vel_lin
    };
    // Process
    const fsb::MotionVector vel_actual = fsb::motion_transform_velocity(tr_parent, vel_parent, tr_parent_child, vel_motion);

    REQUIRE(vel_actual.linear.x == FsbApprox(vel_expected.linear.x));
    REQUIRE(vel_actual.linear.y == FsbApprox(vel_expected.linear.y));
    REQUIRE(vel_actual.linear.z == FsbApprox(vel_expected.linear.z));
    REQUIRE(vel_actual.angular.x == FsbApprox(vel_expected.angular.x));
    REQUIRE(vel_actual.angular.y == FsbApprox(vel_expected.angular.y));
    REQUIRE(vel_actual.angular.z == FsbApprox(vel_expected.angular.z));
}

TEST_CASE("Motion transform acceleration coriolis" * doctest::description("[fsb_motion][fsb::motion_transform_acceleration]"))
{
    const fsb::Vec3 parent_position = {0.12, -0.34, 0.921};
    const fsb::Quaternion parent_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
    const fsb::Vec3 parent_vel_ang = {0.123, -0.2, 0.2432};
    const fsb::Vec3 parent_vel_lin = {-0.9, 0.62, 0.89};
    const fsb::Vec3 parent_acc_ang = {0, 0, 0};
    const fsb::Vec3 parent_acc_lin = {0, 0, 0};

    const fsb::Vec3 motion_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion motion_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::Vec3 motion_vel_ang = {0.1298, -0.723, -1.2};
    const fsb::Vec3 motion_vel_lin = {1.324, 0.555, -0.112};
    const fsb::Vec3 motion_acc_ang = {0, 0, 0};
    const fsb::Vec3 motion_acc_lin = {0, 0, 0};

    const fsb::Transform tr_parent = {
        parent_rotation,
        parent_position
    };
    const fsb::MotionVector vel_parent = {
        parent_vel_ang,
        parent_vel_lin
    };
    const fsb::MotionVector acc_parent = {
        parent_acc_ang,
        parent_acc_lin
    };
    const fsb::Transform tr_motion = {
        motion_rotation,
        motion_position
    };
    const fsb::MotionVector vel_motion = {
        motion_vel_ang,
        motion_vel_lin
    };
    const fsb::MotionVector acc_motion = {
        motion_acc_ang,
        motion_acc_lin
    };
    // Expected
    const fsb::Vec3 expected_acc_ang = {0.34877255915564176, 0.14180290331250361, -0.05977978665149429};
    const fsb::Vec3 expected_acc_lin = {-0.43138808811209683, -0.1522767248619734, 0.09294979385441515};
    const fsb::MotionVector acc_expected = {
        expected_acc_ang,
        expected_acc_lin
    };
    // Process
    const fsb::MotionVector acc_actual = fsb::motion_transform_acceleration(tr_parent, vel_parent, acc_parent, tr_motion, vel_motion, acc_motion);

    REQUIRE(acc_actual.angular.x == FsbApprox(acc_expected.angular.x));
    REQUIRE(acc_actual.angular.y == FsbApprox(acc_expected.angular.y));
    REQUIRE(acc_actual.angular.z == FsbApprox(acc_expected.angular.z));
    REQUIRE(acc_actual.linear.x == FsbApprox(acc_expected.linear.x));
    REQUIRE(acc_actual.linear.y == FsbApprox(acc_expected.linear.y));
    REQUIRE(acc_actual.linear.z == FsbApprox(acc_expected.linear.z));
}

TEST_CASE("Motion transform acceleration full" * doctest::description("[fsb_motion][fsb::motion_transform_acceleration]"))
{
    const fsb::Vec3 parent_position = {0.12, -0.34, 0.921};
    const fsb::Quaternion parent_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
    const fsb::Vec3 parent_vel_ang = {0.123, -0.2, 0.2432};
    const fsb::Vec3 parent_vel_lin = {-0.9, 0.62, 0.89};
    const fsb::Vec3 parent_acc_ang = {0.8268, 0.2647, -0.8049};
    const fsb::Vec3 parent_acc_lin = {-0.443, 0.0938, 0.915};

    const fsb::Vec3 motion_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion motion_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::Vec3 motion_vel_ang = {0.1298, -0.723, -1.2};
    const fsb::Vec3 motion_vel_lin = {1.324, 0.555, -0.112};
    const fsb::Vec3 motion_acc_ang = {0.9298, -0.6848, 0.9412};
    const fsb::Vec3 motion_acc_lin = {0.9143, -0.0292, 0.6006};

    const fsb::Transform tr_parent = {
        parent_rotation,
        parent_position
    };
    const fsb::MotionVector vel_parent = {
        parent_vel_ang,
        parent_vel_lin
    };
    const fsb::MotionVector acc_parent = {
        parent_acc_ang,
        parent_acc_lin
    };
    const fsb::Transform tr_motion = {
        motion_rotation,
        motion_position
    };
    const fsb::MotionVector vel_motion = {
        motion_vel_ang,
        motion_vel_lin
    };
    const fsb::MotionVector acc_motion = {
        motion_acc_ang,
        motion_acc_lin
    };
    // Expected
    const fsb::Vec3 expected_acc_ang = {2.3190543790648785, 1.3588543675783367, -0.7953073422099451};
    const fsb::Vec3 expected_acc_lin = {-0.6722899195542826, 1.0177844331897168, 0.9830403630149204};
    const fsb::MotionVector acc_expected = {
        expected_acc_ang,
        expected_acc_lin
    };
    // Process
    const fsb::MotionVector acc_actual = fsb::motion_transform_acceleration(tr_parent, vel_parent, acc_parent, tr_motion, vel_motion, acc_motion);

    // Check
    const fsb::real_t acc_eps = 1.0e4 * std::numeric_limits<fsb::real_t>::epsilon();
    REQUIRE(acc_actual.linear.x == FsbApprox(acc_expected.linear.x, acc_eps));
    REQUIRE(acc_actual.linear.y == FsbApprox(acc_expected.linear.y, acc_eps));
    REQUIRE(acc_actual.linear.z == FsbApprox(acc_expected.linear.z, acc_eps));
    REQUIRE(acc_actual.angular.x == FsbApprox(acc_expected.angular.x));
    REQUIRE(acc_actual.angular.y == FsbApprox(acc_expected.angular.y));
    REQUIRE(acc_actual.angular.z == FsbApprox(acc_expected.angular.z));
}

TEST_CASE("Vector addition" * doctest::description("[fsb_motion][fsb::vector_add]"))
{
    // Inputs
    const fsb::Vec3 vec_a = {0.12, -0.45, 1.3};
    const fsb::Vec3 vec_b = {-0.45, 0.03, 0.98};
    // Expected
    const fsb::Vec3 v_expected = {-0.33, -0.42, 2.28};
    // Process
    const fsb::Vec3 v_actual = fsb::vector_add(vec_a, vec_b);

    REQUIRE(v_actual.x == FsbApprox(v_expected.x));
    REQUIRE(v_actual.y == FsbApprox(v_expected.y));
    REQUIRE(v_actual.z == FsbApprox(v_expected.z));
}

TEST_CASE("Vector subtraction" * doctest::description("[fsb_motion][fsb::vector_subtract]"))
{
    // Inputs
    const fsb::Vec3 vec_a = {0.12, -0.45, 1.3};
    const fsb::Vec3 vec_b = {-0.45, 0.03, 0.98};
    // Expected
    const fsb::Vec3 v_expected = {0.57, -0.48, 0.32};
    // Process
    const fsb::Vec3 v_actual = fsb::vector_subtract(vec_a, vec_b);

    REQUIRE(v_actual.x == FsbApprox(v_expected.x));
    REQUIRE(v_actual.y == FsbApprox(v_expected.y));
    REQUIRE(v_actual.z == FsbApprox(v_expected.z));
}

TEST_CASE("Cross product" * doctest::description("[fsb_motion][fsb::vector_cross]"))
{
    // Inputs
    const fsb::Vec3 vec_a = {0.12, -0.45, 1.3};
    const fsb::Vec3 vec_b = {-0.45, 0.03, 0.98};
    // Expected
    const fsb::Vec3 v_expected = {-0.48, -0.7026, -0.1989};
    // Process
    const fsb::Vec3 v_actual = fsb::vector_cross(vec_a, vec_b);

    REQUIRE(v_actual.x == FsbApprox(v_expected.x));
    REQUIRE(v_actual.y == FsbApprox(v_expected.y));
    REQUIRE(v_actual.z == FsbApprox(v_expected.z));
}

TEST_CASE("Dot product" * doctest::description("[fsb_motion][fsb::vector_dot]"))
{
    // Inputs
    const fsb::Vec3 vec_a = {0.12, -0.45, 1.3};
    const fsb::Vec3 vec_b = {-0.45, 0.03, 0.98};
    // Expected
    const fsb::real_t s_expected = 1.2065;
    // Process
    const fsb::real_t s_actual = fsb::vector_dot(vec_a, vec_b);

    REQUIRE(s_actual == FsbApprox(s_expected));
}

TEST_CASE("Apply body offset" * doctest::description("[fsb_motion][fsb::coord_transform_apply_body_offset]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    const fsb::MotionVector offset = {
        {0.075, -0.12, 0.56},
        {-0.982, 1.234, -0.9987}
    };
    // Expected
    const fsb::Transform tr_expected = {
        {0.12879363305026925, -0.14346389269453994, -0.8317266006042587, 0.5206353555281533},
        {-0.882, 1.014, 2.1213}
    };
    // Process
    const fsb::Transform tr_actual = fsb::coord_transform_apply_body_offset(tr_a, offset);
    const fsb::MotionVector offset_actual = fsb::coord_transform_get_body_offset(tr_a, tr_actual);

    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));

    REQUIRE(offset_actual.angular.x == FsbApprox(offset.angular.x));
    REQUIRE(offset_actual.angular.y == FsbApprox(offset.angular.y));
    REQUIRE(offset_actual.angular.z == FsbApprox(offset.angular.z));
    REQUIRE(offset_actual.linear.x == FsbApprox(offset.linear.x));
    REQUIRE(offset_actual.linear.y == FsbApprox(offset.linear.y));
    REQUIRE(offset_actual.linear.z == FsbApprox(offset.linear.z));
}

TEST_CASE("Get body offset" * doctest::description("[fsb_motion][fsb::coord_transform_get_body_offset]"))
{
    // Inputs
    const fsb::Transform tr_a = {
        {0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929},
        {0.1, -0.22, 3.12}
    };
    const fsb::Transform tr_b = {
        {0.12879363305026925, -0.14346389269453994, -0.8317266006042587, 0.5206353555281533},
        {-0.882, 1.014, 2.1213}
    };
    // Expected
    const fsb::MotionVector offset_expected = {
        {0.075, -0.12, 0.56},
        {-0.982, 1.234, -0.9987}
    };
    // Process
    const fsb::MotionVector offset_actual = fsb::coord_transform_get_body_offset(tr_a, tr_b);
    const fsb::Transform tr_b_actual = fsb::coord_transform_apply_body_offset(tr_a, offset_actual);

    REQUIRE(offset_actual.angular.x == FsbApprox(offset_expected.angular.x));
    REQUIRE(offset_actual.angular.y == FsbApprox(offset_expected.angular.y));
    REQUIRE(offset_actual.angular.z == FsbApprox(offset_expected.angular.z));
    REQUIRE(offset_actual.linear.x == FsbApprox(offset_expected.linear.x));
    REQUIRE(offset_actual.linear.y == FsbApprox(offset_expected.linear.y));
    REQUIRE(offset_actual.linear.z == FsbApprox(offset_expected.linear.z));

    REQUIRE(fsb::quat_norm(tr_b_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_b_actual.rotation.qw == FsbApprox(tr_b.rotation.qw));
    REQUIRE(tr_b_actual.rotation.qx == FsbApprox(tr_b.rotation.qx));
    REQUIRE(tr_b_actual.rotation.qy == FsbApprox(tr_b.rotation.qy));
    REQUIRE(tr_b_actual.rotation.qz == FsbApprox(tr_b.rotation.qz));

    REQUIRE(tr_b_actual.translation.x == FsbApprox(tr_b.translation.x));
    REQUIRE(tr_b_actual.translation.y == FsbApprox(tr_b.translation.y));
    REQUIRE(tr_b_actual.translation.z == FsbApprox(tr_b.translation.z));
}

// TEST_CASE("Motion body-fixed transform velocity" * doctest::description("[fsb_motion][fsb::motion_transform_body_velocity]"))
// {
//     const fsb::Vec3 parent_position = {0.12, -0.34, 0.921};
//     const fsb::Quaternion parent_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
//     const fsb::Vec3 parent_vel_ang = {0.123, -0.2, 0.2432};
//     const fsb::Vec3 parent_vel_lin = {-0.9, 0.62, 0.89};
//
//     const fsb::Vec3 motion_position = {-0.872, 1.235, -0.02};
//     const fsb::Quaternion motion_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
//     const fsb::Vec3 motion_vel_ang = {0.1298, -0.723, -1.2};
//     const fsb::Vec3 motion_vel_lin = {1.324, 0.555, -0.112};
//
//     const fsb::Transform tr_parent = {
//         parent_rotation,
//         parent_position
//     };
//     const fsb::MotionVector vel_parent = {
//         parent_vel_ang,
//         parent_vel_lin
//     };
//     const fsb::Transform tr_parent_child = {
//         motion_rotation,
//         motion_position
//     };
//     const fsb::MotionVector vel_motion = {
//         motion_vel_ang,
//         motion_vel_lin
//     };
//
//     // Expected
//     const fsb::Vec3 expected_vel_ang = {0.150370037146561, -0.538662500971422, -0.9173840060724154};
//     const fsb::Vec3 expected_vel_lin = {-1.2157473455309495, 1.6384637720685578, 0.6523480454715161};
//     const fsb::MotionVector vel_expected = {
//         expected_vel_ang,
//         expected_vel_lin
//     };
//     // Process
//     const fsb::MotionVector vel_actual = fsb::motion_transform_body_velocity(tr_parent, vel_parent, tr_parent_child, vel_motion);
//
//     REQUIRE(vel_actual.angular.x == FsbApprox(vel_expected.angular.x));
//     REQUIRE(vel_actual.angular.y == FsbApprox(vel_expected.angular.y));
//     REQUIRE(vel_actual.angular.z == FsbApprox(vel_expected.angular.z));
//     REQUIRE(vel_actual.linear.x == FsbApprox(vel_expected.linear.x));
//     REQUIRE(vel_actual.linear.y == FsbApprox(vel_expected.linear.y));
//     REQUIRE(vel_actual.linear.z == FsbApprox(vel_expected.linear.z));
// }
//
// TEST_CASE("Motion body-fixed transform acceleration coriolis" * doctest::description("[fsb_motion][fsb::motion_transform_body_acceleration]"))
// {
//     const fsb::Vec3 parent_position = {0.12, -0.34, 0.921};
//     const fsb::Quaternion parent_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
//     const fsb::Vec3 parent_vel_ang = {0.123, -0.2, 0.2432};
//     const fsb::Vec3 parent_vel_lin = {-0.9, 0.62, 0.89};
//     const fsb::Vec3 parent_acc_ang = {0, 0, 0};
//     const fsb::Vec3 parent_acc_lin = {0, 0, 0};
//
//     const fsb::Vec3 motion_position = {-0.872, 1.235, -0.02};
//     const fsb::Quaternion motion_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
//     const fsb::Vec3 motion_vel_ang = {0.1298, -0.723, -1.2};
//     const fsb::Vec3 motion_vel_lin = {1.324, 0.555, -0.112};
//     const fsb::Vec3 motion_acc_ang = {0, 0, 0};
//     const fsb::Vec3 motion_acc_lin = {0, 0, 0};
//
//     const fsb::Transform tr_parent = {
//         parent_rotation,
//         parent_position
//     };
//     const fsb::MotionVector vel_parent = {
//         parent_vel_ang,
//         parent_vel_lin
//     };
//     const fsb::MotionVector acc_parent = {
//         parent_acc_ang,
//         parent_acc_lin
//     };
//     const fsb::Transform tr_motion = {
//         motion_rotation,
//         motion_position
//     };
//     const fsb::MotionVector vel_motion = {
//         motion_vel_ang,
//         motion_vel_lin
//     };
//     const fsb::MotionVector acc_motion = {
//         motion_acc_ang,
//         motion_acc_lin
//     };
//     // Expected
//     const fsb::Vec3 expected_acc_ang = {-0.016873635224649162, 0.06136760058767289, -0.0387991442308725};
//     const fsb::Vec3 expected_acc_lin = {-0.15612116067423762, 0.10232536915051982, 0.8332402283013183};
//     const fsb::MotionVector acc_expected = {
//         expected_acc_ang,
//         expected_acc_lin
//     };
//     // Process
//     const fsb::MotionVector acc_actual = fsb::motion_transform_body_acceleration(tr_parent, vel_parent, acc_parent, tr_motion, vel_motion, acc_motion);
//
//     REQUIRE(acc_actual.angular.x == FsbApprox(acc_expected.angular.x));
//     REQUIRE(acc_actual.angular.y == FsbApprox(acc_expected.angular.y));
//     REQUIRE(acc_actual.angular.z == FsbApprox(acc_expected.angular.z));
//     REQUIRE(acc_actual.linear.x == FsbApprox(acc_expected.linear.x));
//     REQUIRE(acc_actual.linear.y == FsbApprox(acc_expected.linear.y));
//     REQUIRE(acc_actual.linear.z == FsbApprox(acc_expected.linear.z));
// }
//
// TEST_CASE("Motion body-fixed transform acceleration full" * doctest::description("[fsb_motion][fsb::motion_transform_body_acceleration]"))
// {
//     const fsb::Vec3 parent_position = {0.12, -0.34, 0.921};
//     const fsb::Quaternion parent_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
//     const fsb::Vec3 parent_vel_ang = {0.123, -0.2, 0.2432};
//     const fsb::Vec3 parent_vel_lin = {-0.9, 0.62, 0.89};
//     const fsb::Vec3 parent_acc_ang = {0.8268, 0.2647, -0.8049};
//     const fsb::Vec3 parent_acc_lin = {-0.443, 0.0938, 0.915};
//
//     const fsb::Vec3 motion_position = {-0.872, 1.235, -0.02};
//     const fsb::Quaternion motion_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
//     const fsb::Vec3 motion_vel_ang = {0.1298, -0.723, -1.2};
//     const fsb::Vec3 motion_vel_lin = {1.324, 0.555, -0.112};
//     const fsb::Vec3 motion_acc_ang = {0.9298, -0.6848, 0.9412};
//     const fsb::Vec3 motion_acc_lin = {0.9143, -0.0292, 0.6006};
//
//     const fsb::Transform tr_parent = {
//         parent_rotation,
//         parent_position
//     };
//     const fsb::MotionVector vel_parent = {
//         parent_vel_ang,
//         parent_vel_lin
//     };
//     const fsb::MotionVector acc_parent = {
//         parent_acc_ang,
//         parent_acc_lin
//     };
//     const fsb::Transform tr_motion = {
//         motion_rotation,
//         motion_position
//     };
//     const fsb::MotionVector vel_motion = {
//         motion_vel_ang,
//         motion_vel_lin
//     };
//     const fsb::MotionVector acc_motion = {
//         motion_acc_ang,
//         motion_acc_lin
//     };
//     // Expected
//     const fsb::Vec3 expected_acc_ang = {0.9257911486459605, -1.783537716537328, 1.138024476219782};
//     const fsb::Vec3 expected_acc_lin = {0.002908575283119441, 2.577428624215808, 2.97110924623056};
//     const fsb::MotionVector acc_expected = {
//         expected_acc_ang,
//         expected_acc_lin
//     };
//     // Process
//     const fsb::MotionVector acc_actual = fsb::motion_transform_body_acceleration(tr_parent, vel_parent, acc_parent, tr_motion, vel_motion, acc_motion);
//
//     // Check
//     const fsb::real_t acc_eps = 1.0e4 * std::numeric_limits<fsb::real_t>::epsilon();
//     REQUIRE(acc_actual.angular.x == FsbApprox(acc_expected.angular.x));
//     REQUIRE(acc_actual.angular.y == FsbApprox(acc_expected.angular.y));
//     REQUIRE(acc_actual.angular.z == FsbApprox(acc_expected.angular.z));
//     REQUIRE(acc_actual.linear.x == FsbApprox(acc_expected.linear.x, acc_eps));
//     REQUIRE(acc_actual.linear.y == FsbApprox(acc_expected.linear.y, acc_eps));
//     REQUIRE(acc_actual.linear.z == FsbApprox(acc_expected.linear.z, acc_eps));
// }

TEST_SUITE_END();
