
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_joint.h"

TEST_SUITE_BEGIN("joint");

static void check_fsb_motion_vector(const fsb::MotionVector& v_actual, const fsb::MotionVector& v_expected, const fsb::real_t eps = FsbApprox::default_epsilon)
{
    REQUIRE(v_actual.angular.x == FsbApprox(v_expected.angular.x, eps));
    REQUIRE(v_actual.angular.y == FsbApprox(v_expected.angular.y, eps));
    REQUIRE(v_actual.angular.z == FsbApprox(v_expected.angular.z, eps));

    REQUIRE(v_actual.linear.x == FsbApprox(v_expected.linear.x, eps));
    REQUIRE(v_actual.linear.y == FsbApprox(v_expected.linear.y, eps));
    REQUIRE(v_actual.linear.z == FsbApprox(v_expected.linear.z, eps));
}

static void check_fsb_transform(const fsb::Transform& tr_actual, const fsb::Transform& tr_expected)
{
    REQUIRE(fsb::quat_norm(tr_actual.rotation) == FsbApprox(1.0));
    REQUIRE(tr_actual.rotation.qw == FsbApprox(tr_expected.rotation.qw));
    REQUIRE(tr_actual.rotation.qx == FsbApprox(tr_expected.rotation.qx));
    REQUIRE(tr_actual.rotation.qy == FsbApprox(tr_expected.rotation.qy));
    REQUIRE(tr_actual.rotation.qz == FsbApprox(tr_expected.rotation.qz));

    REQUIRE(tr_actual.translation.x == FsbApprox(tr_expected.translation.x));
    REQUIRE(tr_actual.translation.y == FsbApprox(tr_expected.translation.y));
    REQUIRE(tr_actual.translation.z == FsbApprox(tr_expected.translation.z));
}

TEST_CASE(
    "Fixed joint pva"
    * doctest::description("[fsb_joint][fsb::joint_parent_child_pva][fsb::joint_parent_child_velocity][fsb::joint_parent_child_transform]"))
{
    // Inputs
    const fsb::Joint joint = {
        fsb::JointType::FIXED,
        {{0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929}, {0.1, -0.22, 3.12}},
        {},
        0U,
        0U,
        0U,
        0U
    };
    const fsb::JointPva joint_pva = {};
    // Expected
    const fsb::CartesianPva pva_expected{
        {{0.311126983722081, 0.0565685424949238, -0.848528137423857, 0.424264068711929}, {0.1, -0.22, 3.12}},
        {},
        {}
    };
    // Process
    const fsb::CartesianPva pva_actual = fsb::joint_parent_child_pva(joint, joint_pva);

    SUBCASE("Check Cartesian pva pose")
    {
        check_fsb_transform(pva_actual.pose, pva_expected.pose);
    }
    SUBCASE("Check Cartesian velocity")
    {
        check_fsb_motion_vector(pva_actual.velocity, pva_expected.velocity);
    }
    SUBCASE("Check Cartesian acceleration")
    {
        check_fsb_motion_vector(pva_actual.acceleration, pva_expected.acceleration);
    }
}

TEST_CASE(
    "Revolute joint pva"
    * doctest::description("[fsb_joint][fsb::joint_parent_child_pva][fsb::joint_parent_child_velocity][fsb::joint_parent_child_transform]"))
{

    // Inputs
    const auto joint_type = fsb::JointType::REVOLUTE_Z;

    const fsb::Vec3 joint_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion joint_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::real_t joint_qpos = 0.45;
    const fsb::real_t joint_qvel = -0.5;
    const fsb::real_t joint_qacc = 1.5;

    // Expected
    const fsb::Vec3 expected_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion expected_rotation = {0.42726409489091893, 0.5815846003814831, -0.036735998202557636, 0.6912707228374538};
    const fsb::Vec3 expected_vel_ang = {-0.38633643407490936, 0.27388473791612855, -0.16040981903517226};
    const fsb::Vec3 expected_vel_lin = {0, 0, 0};
    const fsb::Vec3 expected_acc_ang = {1.159009302224728, -0.8216542137483857, 0.4812294571055168};
    const fsb::Vec3 expected_acc_lin = {0, 0, 0};
    const fsb::CartesianPva pva_expected{
        {expected_rotation, expected_position},
        {expected_vel_ang, expected_vel_lin},
        {expected_acc_ang, expected_acc_lin}
    };

    // Process
    const fsb::Joint joint = {
        joint_type,
        {
            joint_rotation,
            joint_position},
        {},
        0U,
        0U,
        0U,
        0U
    };
    fsb::JointPva joint_pva = {};
    joint_pva.position.q[0] = joint_qpos;
    joint_pva.velocity.qv[0] = joint_qvel;
    joint_pva.acceleration.qv[0] = joint_qacc;

    // call joint_parent_child_pva
    const fsb::CartesianPva pva_actual = fsb::joint_parent_child_pva(joint, joint_pva);

    SUBCASE("Check Cartesian pose")
    {
        check_fsb_transform(pva_actual.pose, pva_expected.pose);
    }
    SUBCASE("Check Cartesian velocity")
    {
        check_fsb_motion_vector(pva_actual.velocity, pva_expected.velocity);
    }
    SUBCASE("Check Cartesian acceleration")
    {
        check_fsb_motion_vector(pva_actual.acceleration, pva_expected.acceleration);
    }
}

TEST_CASE(
    "Prismatic joint pva"
    * doctest::description("[fsb_joint][fsb::joint_parent_child_pva][fsb::joint_parent_child_velocity][fsb::joint_parent_child_transform]"))
{
    // Inputs
    const fsb::Joint joint = {
        fsb::JointType::PRISMATIC_Z,
        {{0.8658406131180871, -0.2843509043081139, -0.373283005655497, 0.17356380262961224}, {0.5, 0.1, -0.05}},
        {},
        0U,
        0U,
        0U,
        0U
    };
    fsb::JointPva joint_pva = {};
    joint_pva.position.q[0] = 0.2;
    joint_pva.velocity.qv[0] = 0.4;
    joint_pva.acceleration.qv[0] = -0.5;
    // Expected
    const fsb::CartesianPva pva_expected{
        {{0.8658406131180871, -0.2843509043081139, -0.373283005655497, 0.17356380262961224},
         {0.35097735571351873, 0.1725656573632963, 0.06192174436318241}},
        {{0.0, 0.0, 0.0}, {-0.29804528857296253, 0.14513131472659258, 0.22384348872636484}},
        {{0.0, 0.0, 0.0}, {0.3725566107162032, -0.18141414340824075, -0.2798043609079561} }
    };
    // Process
    const fsb::CartesianPva pva_actual = fsb::joint_parent_child_pva(joint, joint_pva);

    SUBCASE("Check Cartesian pva pose")
    {
        check_fsb_transform(pva_actual.pose, pva_expected.pose);
    }
    SUBCASE("Check Cartesian velocity")
    {
        check_fsb_motion_vector(pva_actual.velocity, pva_expected.velocity);
    }
    SUBCASE("Check Cartesian acceleration")
    {
        check_fsb_motion_vector(pva_actual.acceleration, pva_expected.acceleration);
    }
}

TEST_CASE(
    "Spherical joint pva"
    * doctest::description("[fsb_joint][fsb::joint_parent_child_pva][fsb::joint_parent_child_velocity][fsb::joint_parent_child_transform]"))
{
    // Inputs
    const fsb::real_t eps = 2.0 * FsbApprox::default_epsilon;
    const auto joint_type = fsb::JointType::SPHERICAL;

    const fsb::Vec3 joint_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion joint_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::Quaternion joint_qpos = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
    const fsb::Vec3 joint_qvel = {-0.65, 0.1, 0.883};
    const fsb::Vec3 joint_qacc = {0.95, -0.111, 1.52};

    const fsb::Vec3 expected_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion expected_rotation = {-0.04257094929073869, 0.3492179394988832, -0.044825919143307066, 0.9350000973163575};
    const fsb::Vec3 expected_vel_ang = {0.42360757485867934, -1.0162363984384124, -0.003033976149862485};
    const fsb::Vec3 expected_vel_lin = {0, -0, 0};
    const fsb::Vec3 expected_acc_ang = {1.5330929603824746, -0.06589110115893787, 0.9330087553789493};
    const fsb::Vec3 expected_acc_lin = {0, -0, 0};

    const fsb::CartesianPva pva_expected{
        {expected_rotation, expected_position},
        {expected_vel_ang, expected_vel_lin},
        {expected_acc_ang, expected_acc_lin}
    };

    // Process
    const fsb::Joint joint = {
        joint_type,
        {
            joint_rotation,
            joint_position},
        {},
        0U,
        0U,
        0U,
        0U
    };
    fsb::JointPva joint_pva = {};
    joint_pva.position.q[0] = joint_qpos.qw;
    joint_pva.position.q[1] = joint_qpos.qx;
    joint_pva.position.q[2] = joint_qpos.qy;
    joint_pva.position.q[3] = joint_qpos.qz;

    joint_pva.velocity.qv[0] = joint_qvel.x;
    joint_pva.velocity.qv[1] = joint_qvel.y;
    joint_pva.velocity.qv[2] = joint_qvel.z;

    joint_pva.acceleration.qv[0] = joint_qacc.x;
    joint_pva.acceleration.qv[1] = joint_qacc.y;
    joint_pva.acceleration.qv[2] = joint_qacc.z;

    // Run joint_parent_child_pva
    const fsb::CartesianPva pva_actual = fsb::joint_parent_child_pva(joint, joint_pva);

    SUBCASE("Check Cartesian pva pose")
    {
        check_fsb_transform(pva_actual.pose, pva_expected.pose);
    }
    SUBCASE("Check Cartesian velocity")
    {
        check_fsb_motion_vector(pva_actual.velocity, pva_expected.velocity, eps);
    }
    SUBCASE("Check Cartesian acceleration")
    {
        check_fsb_motion_vector(pva_actual.acceleration, pva_expected.acceleration, eps);
    }
}

TEST_CASE(
    "Cartesian 6DoF joint pva"
    * doctest::description("[fsb_joint][fsb::joint_parent_child_pva][fsb::joint_parent_child_velocity][fsb::joint_parent_child_transform]"))
{
    // Inputs
    const fsb::Quaternion parent_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::Vec3 parent_position = {-0.872, 1.235, -0.02};

    const fsb::Quaternion joint_qpos_ang = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
    const fsb::Vec3 joint_qpos_lin = {0.12, -0.34, 0.921};
    const fsb::Vec3 joint_qvel_ang = {0.123, -0.2, 0.2432};
    const fsb::Vec3 joint_qvel_lin = {-0.9, 0.62, 0.89};
    const fsb::Vec3 joint_qacc_ang = {0.8268, 0.2647, -0.8049};
    const fsb::Vec3 joint_qacc_lin = {-0.443, 0.0938, 0.915};

    const fsb::Vec3 expected_position = {0.06496702905780316, 0.9352202887655513, 0.08230303607803631};
    const fsb::Quaternion expected_rotation = {0.04257094929073869, -0.3492179394988832, 0.044825919143307066, -0.9350000973163575};
    const fsb::Vec3 expected_vel_ang = {0.336867791895353, 0.027477283706372292, -0.006350564043395023};
    const fsb::Vec3 expected_vel_lin = {0.06358217773305644, -1.3842423875879208, 0.25754673183641935};
    const fsb::Vec3 expected_acc_ang = {-0.5093477531243327, 0.9886300418345246, 0.405837217082745};
    const fsb::Vec3 expected_acc_lin = {0.5165430914384911, -0.8726528163866043, 0.11803701427812974};

    const fsb::Transform tr_parent = {parent_rotation, parent_position};
    const fsb::Joint joint = {
        fsb::JointType::CARTESIAN,
        tr_parent, {},
        0U, 0U, 0U, 0U
    };
    fsb::JointPva joint_pva = {};

    joint_pva.position.q[0] = joint_qpos_ang.qw;
    joint_pva.position.q[1] = joint_qpos_ang.qx;
    joint_pva.position.q[2] = joint_qpos_ang.qy;
    joint_pva.position.q[3] = joint_qpos_ang.qz;
    joint_pva.position.q[4] = joint_qpos_lin.x;
    joint_pva.position.q[5] = joint_qpos_lin.y;
    joint_pva.position.q[6] = joint_qpos_lin.z;

    joint_pva.velocity.qv[0] = joint_qvel_ang.x;
    joint_pva.velocity.qv[1] = joint_qvel_ang.y;
    joint_pva.velocity.qv[2] = joint_qvel_ang.z;
    joint_pva.velocity.qv[3] = joint_qvel_lin.x;
    joint_pva.velocity.qv[4] = joint_qvel_lin.y;
    joint_pva.velocity.qv[5] = joint_qvel_lin.z;

    joint_pva.acceleration.qv[0] = joint_qacc_ang.x;
    joint_pva.acceleration.qv[1] = joint_qacc_ang.y;
    joint_pva.acceleration.qv[2] = joint_qacc_ang.z;
    joint_pva.acceleration.qv[3] = joint_qacc_lin.x;
    joint_pva.acceleration.qv[4] = joint_qacc_lin.y;
    joint_pva.acceleration.qv[5] = joint_qacc_lin.z;

    // Expected
    const fsb::CartesianPva pva_expected{
        {expected_rotation, expected_position},
        {expected_vel_ang, expected_vel_lin},
        {expected_acc_ang, expected_acc_lin}
    };

    // Process
    const fsb::CartesianPva pva_actual = fsb::joint_parent_child_pva(joint, joint_pva);

    SUBCASE("Check Cartesian pva pose")
    {
        check_fsb_transform(pva_actual.pose, pva_expected.pose);
    }
    SUBCASE("Check Cartesian velocity")
    {
        check_fsb_motion_vector(pva_actual.velocity, pva_expected.velocity);
    }
    SUBCASE("Check Cartesian acceleration")
    {
        check_fsb_motion_vector(pva_actual.acceleration, pva_expected.acceleration);
    }
}

TEST_SUITE_END();
