
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_kinematics.h"
#include "fsb_body_tree_sample.h"
#include "fsb_rotation.h"

TEST_SUITE_BEGIN("kinematics");

TEST_CASE("Forward Kinematics RPR fixed base" * doctest::description("[fsb_kinematics][fsb::forward_kinematics]"))
{
    // Inputs
    const fsb::Vec3 base_position = {0.12, -0.34, 0.921};
    const fsb::Quaternion base_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
    const fsb::Vec3 base_vel_ang = {0, 0, 0};
    const fsb::Vec3 base_vel_lin = {0, 0, 0};
    const fsb::Vec3 base_acc_ang = {0, 0, 0};
    const fsb::Vec3 base_acc_lin = {0, 0, 0};

    const fsb::Vec3 joint1_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion joint1_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::real_t joint1_qpos = 0.45;
    const fsb::real_t joint1_qvel = -0.5;
    const fsb::real_t joint1_qacc = 1.5;

    const fsb::Vec3 joint2_position = {0, 0, 0};
    const fsb::Quaternion joint2_rotation = {0.466361491477014, -0.571547679819811, -0.124868616337094, 0.663511897115633};
    const fsb::real_t joint2_qpos = 1.73;
    const fsb::real_t joint2_qvel = 0.71;
    const fsb::real_t joint2_qacc = 1.03;

    const fsb::Vec3 joint3_position = {0, 0, 0};
    const fsb::Quaternion joint3_rotation = {0.461283965309215, 0.607100248856612, 0.205771002489209, -0.613436782171915};
    const fsb::real_t joint3_qpos = 0.97;
    const fsb::real_t joint3_qvel = -0.43;
    const fsb::real_t joint3_qacc = 0.62;

    // Expected
    const fsb::Vec3 expected_position = {-0.1609681549732388, -1.4630302404666433, 0.40345719786411616};
    const fsb::Quaternion expected_rotation = {0.4514982278748776, -0.7407761160842639, -0.022397671379960134, -0.4968887605709308};
    const fsb::Vec3 expected_vel_ang = {-0.6396719468163807, -0.6420126810046499, 0.18728932762965803};
    const fsb::Vec3 expected_vel_lin = {0.8948150814283379, -0.5969444479315489, -0.1442851922325477};
    const fsb::Vec3 expected_acc_ang = {1.4678054889047247, 1.4323856496043115, -0.5034042391044654};
    const fsb::Vec3 expected_acc_lin = {-0.9068303815731835, 0.705491545658221, -1.5187430734580145};

    // Inputs
    fsb::CartesianPva base_pva = {
        {base_rotation, base_position},
        {base_vel_ang, base_vel_lin},
        {base_acc_ang, base_acc_lin}
    };
    const fsb::Transform joint1_tr = {joint1_rotation, joint1_position};
    const fsb::Transform joint2_tr = {joint2_rotation, joint2_position};
    const fsb::Transform joint3_tr = {joint3_rotation, joint3_position};

    fsb::JointPva joint_pva = {
        {joint1_qpos, joint2_qpos, joint3_qpos},
        {joint1_qvel, joint2_qvel, joint3_qvel},
        {joint1_qacc, joint2_qacc, joint3_qacc}
    };
    // Expected
    fsb::CartesianPva expected_body3_pva = {
    {expected_rotation, expected_position},
    {expected_vel_ang, expected_vel_lin},
        {expected_acc_ang, expected_acc_lin}};
    // Process
    size_t body3_index = 3U;
    fsb::BodyTree body_tree = body_tree_sample_rpr(joint1_tr, joint2_tr, joint3_tr, body3_index);
    fsb::BodyCartesianPva actual_body_pva = {};
    const auto opt = fsb::ForwardKinematicsOption::POSE_VELOCITY_ACCELERATION;
    forward_kinematics(body_tree, joint_pva, base_pva, opt, actual_body_pva);

    // check body 3 PVA
    const auto& [pose, velocity, acceleration] = actual_body_pva.body[body3_index];

    REQUIRE(expected_body3_pva.pose.translation.x == FsbApprox(pose.translation.x));
    REQUIRE(expected_body3_pva.pose.translation.y == FsbApprox(pose.translation.y));
    REQUIRE(expected_body3_pva.pose.translation.z == FsbApprox(pose.translation.z));

    REQUIRE(1.0 == FsbApprox(fsb::quat_norm(pose.rotation)));
    REQUIRE(expected_body3_pva.pose.rotation.qw == FsbApprox(pose.rotation.qw));
    REQUIRE(expected_body3_pva.pose.rotation.qx == FsbApprox(pose.rotation.qx));
    REQUIRE(expected_body3_pva.pose.rotation.qy == FsbApprox(pose.rotation.qy));
    REQUIRE(expected_body3_pva.pose.rotation.qz == FsbApprox(pose.rotation.qz));

    REQUIRE(expected_body3_pva.velocity.linear.x == FsbApprox(velocity.linear.x));
    REQUIRE(expected_body3_pva.velocity.linear.y == FsbApprox(velocity.linear.y));
    REQUIRE(expected_body3_pva.velocity.linear.z == FsbApprox(velocity.linear.z));

    REQUIRE(expected_body3_pva.velocity.angular.x == FsbApprox(velocity.angular.x));
    REQUIRE(expected_body3_pva.velocity.angular.y == FsbApprox(velocity.angular.y));
    REQUIRE(expected_body3_pva.velocity.angular.z == FsbApprox(velocity.angular.z));

    REQUIRE(expected_body3_pva.acceleration.linear.x == FsbApprox(acceleration.linear.x));
    REQUIRE(expected_body3_pva.acceleration.linear.y == FsbApprox(acceleration.linear.y));
    REQUIRE(expected_body3_pva.acceleration.linear.z == FsbApprox(acceleration.linear.z));

    REQUIRE(expected_body3_pva.acceleration.angular.x == FsbApprox(acceleration.angular.x));
    REQUIRE(expected_body3_pva.acceleration.angular.y == FsbApprox(acceleration.angular.y));
    REQUIRE(expected_body3_pva.acceleration.angular.z == FsbApprox(acceleration.angular.z));

}

TEST_CASE("Forward Kinematics RPR moving base" * doctest::description("[fsb_kinematics][fsb::forward_kinematics]"))
{
    // Inputs
    const fsb::Vec3 base_position = {0.12, -0.34, 0.921};
    const fsb::Quaternion base_rotation = {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075};
    const fsb::Vec3 base_vel_ang = {0.123, -0.2, 0.2432};
    const fsb::Vec3 base_vel_lin = {-0.9, 0.62, 0.89};
    const fsb::Vec3 base_acc_ang = {0.8268, 0.2647, -0.8049};
    const fsb::Vec3 base_acc_lin = {-0.443, 0.0938, 0.915};

    const fsb::Vec3 joint1_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion joint1_rotation = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::real_t joint1_qpos = 0.45;
    const fsb::real_t joint1_qvel = -0.5;
    const fsb::real_t joint1_qacc = 1.5;

    const fsb::Vec3 joint2_position = {0.12, 0.05, -0.01};
    const fsb::Quaternion joint2_rotation = {0.466361491477014, -0.571547679819811, -0.124868616337094, 0.663511897115633};
    const fsb::real_t joint2_qpos = 1.73;
    const fsb::real_t joint2_qvel = 0.71;
    const fsb::real_t joint2_qacc = 1.03;

    const fsb::Vec3 joint3_position = {0.1, -0.8, 1.4};
    const fsb::Quaternion joint3_rotation = {0.461283965309215, 0.607100248856612, 0.205771002489209, -0.613436782171915};
    const fsb::real_t joint3_qpos = 0.97;
    const fsb::real_t joint3_qvel = -0.43;
    const fsb::real_t joint3_qacc = 0.62;

    // Expected
    const fsb::Vec3 expected_position = {1.2187293812778743, -1.7511973173084754, -0.3197194105441038};
    const fsb::Quaternion expected_rotation = {0.4514982278748776, -0.7407761160842639, -0.022397671379960134, -0.4968887605709308};
    const fsb::Vec3 expected_vel_ang = {-0.516671946816379, -0.8420126810046509, 0.43048932762965847};
    const fsb::Vec3 expected_vel_lin = {0.8772834262042055, 0.4028771557931141, 1.3632312290011928};
    const fsb::Vec3 expected_acc_ang = {2.413285107399128, 1.5184808448401204, -1.5152061882313153};
    const fsb::Vec3 expected_acc_lin = {-3.8510430470430403, 1.9024397721300113, -3.1741516212151923};

    // Inputs
    fsb::CartesianPva base_pva = {
        {base_rotation, base_position},
        {base_vel_ang, base_vel_lin},
        {base_acc_ang, base_acc_lin}
    };
    const fsb::Transform joint1_tr = {joint1_rotation, joint1_position};
    const fsb::Transform joint2_tr = {joint2_rotation, joint2_position};
    const fsb::Transform joint3_tr = {joint3_rotation, joint3_position};

    fsb::JointPva joint_pva = {
        {joint1_qpos, joint2_qpos, joint3_qpos},
        {joint1_qvel, joint2_qvel, joint3_qvel},
        {joint1_qacc, joint2_qacc, joint3_qacc}
    };
    // Expected
    fsb::CartesianPva expected_body3_pva = {
    {expected_rotation, expected_position},
    {expected_vel_ang, expected_vel_lin},
        {expected_acc_ang, expected_acc_lin}};
    // Process
    size_t body3_index = 3U;
    fsb::BodyTree body_tree = body_tree_sample_rpr(joint1_tr, joint2_tr, joint3_tr, body3_index);
    fsb::BodyCartesianPva actual_body_pva = {};
    const auto opt = fsb::ForwardKinematicsOption::POSE_VELOCITY_ACCELERATION;
    forward_kinematics(body_tree, joint_pva, base_pva, opt, actual_body_pva);

    // check body 3 PVA
    const auto& [pose, velocity, acceleration] = actual_body_pva.body[body3_index];

    REQUIRE(expected_body3_pva.pose.translation.x == FsbApprox(pose.translation.x));
    REQUIRE(expected_body3_pva.pose.translation.y == FsbApprox(pose.translation.y));
    REQUIRE(expected_body3_pva.pose.translation.z == FsbApprox(pose.translation.z));

    REQUIRE(1.0 == FsbApprox(fsb::quat_norm(pose.rotation)));
    REQUIRE(expected_body3_pva.pose.rotation.qw == FsbApprox(pose.rotation.qw));
    REQUIRE(expected_body3_pva.pose.rotation.qx == FsbApprox(pose.rotation.qx));
    REQUIRE(expected_body3_pva.pose.rotation.qy == FsbApprox(pose.rotation.qy));
    REQUIRE(expected_body3_pva.pose.rotation.qz == FsbApprox(pose.rotation.qz));

    REQUIRE(expected_body3_pva.velocity.linear.x == FsbApprox(velocity.linear.x));
    REQUIRE(expected_body3_pva.velocity.linear.y == FsbApprox(velocity.linear.y));
    REQUIRE(expected_body3_pva.velocity.linear.z == FsbApprox(velocity.linear.z));

    REQUIRE(expected_body3_pva.velocity.angular.x == FsbApprox(velocity.angular.x));
    REQUIRE(expected_body3_pva.velocity.angular.y == FsbApprox(velocity.angular.y));
    REQUIRE(expected_body3_pva.velocity.angular.z == FsbApprox(velocity.angular.z));

    REQUIRE(expected_body3_pva.acceleration.linear.x == FsbApprox(acceleration.linear.x));
    REQUIRE(expected_body3_pva.acceleration.linear.y == FsbApprox(acceleration.linear.y));
    REQUIRE(expected_body3_pva.acceleration.linear.z == FsbApprox(acceleration.linear.z));

    REQUIRE(expected_body3_pva.acceleration.angular.x == FsbApprox(acceleration.angular.x));
    REQUIRE(expected_body3_pva.acceleration.angular.y == FsbApprox(acceleration.angular.y));
    REQUIRE(expected_body3_pva.acceleration.angular.z == FsbApprox(acceleration.angular.z));

}

TEST_CASE("Forward Kinematics RPR static" * doctest::description("[fsb_kinematics][fsb::forward_kinematics]"))
{
    const fsb::Vec3 base_position = {0, 0, 0};
    const fsb::Quaternion base_rotation = {1, 0, 0, 0};
    const fsb::Vec3 base_vel_ang = {0, 0, 0};
    const fsb::Vec3 base_vel_lin = {0, 0, 0};
    const fsb::Vec3 base_acc_ang = {0, 0, 0};
    const fsb::Vec3 base_acc_lin = {0, 0, 0};

    const fsb::Vec3 joint1_position = {1.0, 1.0, 0};
    const fsb::Quaternion joint1_rotation = fsb::quat_rx(90.0 * M_PI / 180.0);
    const fsb::real_t joint1_qpos = 90.0 * M_PI / 180.0;
    const fsb::real_t joint1_qvel = 0;
    const fsb::real_t joint1_qacc = 0;

    const fsb::Vec3 joint2_position = {0, 0, 0};
    const fsb::Quaternion joint2_rotation = {1, 0, 0, 0};
    const fsb::real_t joint2_qpos = 2.0;
    const fsb::real_t joint2_qvel = 0;
    const fsb::real_t joint2_qacc = 0;

    const fsb::Vec3 joint3_position = {0, 0, 0};
    const fsb::Quaternion joint3_rotation =  {1, 0, 0, 0};
    const fsb::real_t joint3_qpos = 0.0;
    const fsb::real_t joint3_qvel = 0;
    const fsb::real_t joint3_qacc = 0;

    const fsb::Vec3 expected_position = {1.0, -1.0, 0.0};
    const fsb::Quaternion expected_rotation = {0.5, 0.5, -0.5, 0.5};
    const fsb::Vec3 expected_vel_ang = {0, 0, 0};
    const fsb::Vec3 expected_vel_lin = {0, 0, 0};
    const fsb::Vec3 expected_acc_ang = {0, 0, 0};
    const fsb::Vec3 expected_acc_lin = {0, 0, 0};

    // Inputs
    fsb::CartesianPva base_pva = {
        {base_rotation, base_position},
        {base_vel_ang, base_vel_lin},
        {base_acc_ang, base_acc_lin}
    };
    const fsb::Transform joint1_tr = {joint1_rotation, joint1_position};
    const fsb::Transform joint2_tr = {joint2_rotation, joint2_position};
    const fsb::Transform joint3_tr = {joint3_rotation, joint3_position};

    fsb::JointPva joint_pva = {
        {joint1_qpos, joint2_qpos, joint3_qpos},
        {joint1_qvel, joint2_qvel, joint3_qvel},
        {joint1_qacc, joint2_qacc, joint3_qacc}
    };
    // Expected
    fsb::CartesianPva expected_body3_pva = {
        {expected_rotation, expected_position},
        {expected_vel_ang, expected_vel_lin},
        {expected_acc_ang, expected_acc_lin}};
    // Process
    size_t body3_index = 3U;
    fsb::BodyTree body_tree = body_tree_sample_rpr(joint1_tr, joint2_tr, joint3_tr, body3_index);
    fsb::BodyCartesianPva actual_body_pva = {};
    const auto opt = fsb::ForwardKinematicsOption::POSE_VELOCITY_ACCELERATION;
    forward_kinematics(body_tree, joint_pva, base_pva, opt, actual_body_pva);

    const auto& [pose, velocity, acceleration] = actual_body_pva.body[body3_index];

    REQUIRE(expected_body3_pva.pose.translation.x == FsbApprox(pose.translation.x));
    REQUIRE(expected_body3_pva.pose.translation.y == FsbApprox(pose.translation.y));
    REQUIRE(expected_body3_pva.pose.translation.z == FsbApprox(pose.translation.z));

    REQUIRE(1.0 == FsbApprox(fsb::quat_norm(pose.rotation)));
    REQUIRE(expected_body3_pva.pose.rotation.qw == FsbApprox(pose.rotation.qw));
    REQUIRE(expected_body3_pva.pose.rotation.qx == FsbApprox(pose.rotation.qx));
    REQUIRE(expected_body3_pva.pose.rotation.qy == FsbApprox(pose.rotation.qy));
    REQUIRE(expected_body3_pva.pose.rotation.qz == FsbApprox(pose.rotation.qz));
}

TEST_SUITE_END();
