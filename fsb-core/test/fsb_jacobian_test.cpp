#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_body_tree_sample.h"
#include "fsb_kinematics.h"
#include "fsb_jacobian.h"
#include "fsb_rotation.h"

TEST_SUITE_BEGIN("jacobian");

TEST_CASE("Jacobian RPR" * doctest::description("[fsb_jacobian][fsb::calculate_jacobian]"))
{
    // Inputs
    const fsb::Vec3       joint1_position = {-0.872, 1.235, -0.02};
    const fsb::Quaternion joint1_rotation
        = {0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613};
    const fsb::real_t joint1_qpos = 0.45;

    const fsb::Vec3       joint2_position = {0.12, 0.05, -0.01};
    const fsb::Quaternion joint2_rotation
        = {0.466361491477014, -0.571547679819811, -0.124868616337094, 0.663511897115633};
    const fsb::real_t joint2_qpos = 1.73;

    const fsb::Vec3       joint3_position = {0.1, -0.8, 1.4};
    const fsb::Quaternion joint3_rotation
        = {0.461283965309215, 0.607100248856612, 0.205771002489209, -0.613436782171915};
    const fsb::real_t joint3_qpos = 0.97;

    // Expected
    const fsb::Jacobian expected_jac = {
        {0.7726728681498187,
         -0.5477694758322571,
         0.3208196380703445, 1.34096103517755,
         0.47571437286483415, -2.4173781914425607,
         0, 0,
         0, -0.025345073665008336,
         -0.8845153269080936,
         -0.46582213741468365,
         0.716558267043151, -0.4856522484836865,
         0.5006856733269477, 0.0,
         0.0, 0.0}
    };
    // Process
    fsb::CartesianPva    base_pva = {fsb::transform_identity(), {}, {}};
    const fsb::Transform joint1_tr = {joint1_rotation, joint1_position};
    const fsb::Transform joint2_tr = {joint2_rotation, joint2_position};
    const fsb::Transform joint3_tr = {joint3_rotation, joint3_position};
    fsb::JointPva        joint_pva = {
        {joint1_qpos, joint2_qpos, joint3_qpos},
        {},
        {}
    };

    size_t        body3_index = 0U;
    constexpr fsb::MassProps unit_mass_props = { 1.0, {}, {1.0, 1.0, 1.0, 0.0, 0.0, 0.0}};
    fsb::BodyTree body_tree = body_tree_sample_rpr(
        joint1_tr, joint2_tr, joint3_tr,
        unit_mass_props, unit_mass_props, unit_mass_props,
        body3_index);
    fsb::BodyCartesianPva actual_body_pva = {};
    fsb::Jacobian         actual_jacobian = {};
    const auto            opt = fsb::ForwardKinematicsOption::POSE_VELOCITY_ACCELERATION;
    fsb::forward_kinematics(body_tree, joint_pva, base_pva, opt, actual_body_pva);
    fsb::calculate_jacobian(body3_index, body_tree, actual_body_pva, actual_jacobian);

    // Check body 3 Jacobian
    const size_t num_dofs = body_tree.get_num_dofs();
    for (size_t dof_index = 0; dof_index < num_dofs; ++dof_index)
    {
        for (size_t motion_index = 0; motion_index < 6U; ++motion_index)
        {
            const size_t jac_index = motion_index + dof_index * 6U;
            REQUIRE(actual_jacobian.j[jac_index] == FsbApprox(expected_jac.j[jac_index]));
        }
    }
}

TEST_CASE("Jacobian Panda" * doctest::description("[fsb_jacobian][fsb::calculate_jacobian]"))
{
    // Create the Panda robot's BodyTree
    size_t        ee_index = 0;
    fsb::BodyTree panda_tree = create_panda_body_tree(ee_index);

    // input position and velocity vector
    fsb::JointPva joint_pva = {};
    joint_pva.position.q[0] = 0.0;
    joint_pva.position.q[1] = -0.3;
    joint_pva.position.q[2] = 0.0;
    joint_pva.position.q[3] = -2.2;
    joint_pva.position.q[4] = 0.0;
    joint_pva.position.q[5] = 2.0;
    joint_pva.position.q[6] = 0.79;
    joint_pva.velocity.qv[0] = 0.1;
    joint_pva.velocity.qv[1] = 0.2;
    joint_pva.velocity.qv[2] = 0.3;
    joint_pva.velocity.qv[3] = 0.4;
    joint_pva.velocity.qv[4] = 0.5;
    joint_pva.velocity.qv[5] = 0.6;
    joint_pva.velocity.qv[6] = 0.7;

    // Expected
    fsb::Jacobian jac_expected = {
        {0.0, 0.0, 1.0, 0.0, 0.47372404011176217, 0.0,
        -4.087781603011344e-17, 1.0, 1.2001854874308034e-17, 0.1825132061520504, 7.158809330225282e-17, -0.47372404011176217,
        -0.2955202066613396, -4.6171042683083434e-17, 0.955336489125606, -2.5896082402475335e-17, 0.5065022016952463, -2.2920594548259233e-17,
        2.8895455978778876e-17, -1.0, 1.0742206710881004e-16, 0.14375354146120162, -1.3390616674610668e-18, 0.48829316506388304,
        0.9463000876874146, -3.30191202598639e-17, -0.3232895668635035, -1.9667458475373597e-18, 0.06067390305419941, -6.935392230289308e-18,
        2.8895455978778876e-17, -1.0, 1.0742206710881004e-16, 0.09768010501982789, 7.291639247371775e-18, 0.09824254212567686,
        0.09983341664682802, -1.2032944640052442e-16, -0.9950041652780257, 0.0, 0.0, 0.0}
    };
    fsb::MotionVector vel_expected = {
    {0.45437737349808505, -0.8, -0.4715467523886879},
    {0.15261212082678746, 0.22966001604684982, 0.15951798327860692}};

    // Compute FK
    fsb::CartesianPva base_pva = {};
    base_pva.pose = fsb::transform_identity();
    const fsb::ForwardKinematicsOption opt = fsb::ForwardKinematicsOption::POSE_VELOCITY;
    fsb::BodyCartesianPva              cartesian_pva = {};
    fsb::forward_kinematics(panda_tree, joint_pva, base_pva, opt, cartesian_pva);
    // Compute jacobian
    fsb::Jacobian            jac = {};
    const fsb::JacobianError jac_err
        = fsb::calculate_jacobian(ee_index, panda_tree, cartesian_pva, jac);
    // Velocity from Jacobian
    fsb::MotionVector jac_vel = fsb::jacobian_multiply(jac, joint_pva.velocity, panda_tree.get_num_dofs());

    // Check
    REQUIRE(jac_err == fsb::JacobianError::SUCCESS);
    // compare velocity
    const fsb::Vec3 fk_vel_lin = cartesian_pva.body[ee_index].velocity.linear;
    const fsb::Vec3 fk_vel_ang = cartesian_pva.body[ee_index].velocity.angular;
    REQUIRE(fk_vel_ang.x == FsbApprox(jac_vel.angular.x));
    REQUIRE(fk_vel_ang.y == FsbApprox(jac_vel.angular.y));
    REQUIRE(fk_vel_ang.z == FsbApprox(jac_vel.angular.z));
    REQUIRE(fk_vel_lin.x == FsbApprox(jac_vel.linear.x));
    REQUIRE(fk_vel_lin.y == FsbApprox(jac_vel.linear.y));
    REQUIRE(fk_vel_lin.z == FsbApprox(jac_vel.linear.z));
    REQUIRE(vel_expected.angular.x == FsbApprox(jac_vel.angular.x));
    REQUIRE(vel_expected.angular.y == FsbApprox(jac_vel.angular.y));
    REQUIRE(vel_expected.angular.z == FsbApprox(jac_vel.angular.z));
    REQUIRE(vel_expected.linear.x == FsbApprox(jac_vel.linear.x));
    REQUIRE(vel_expected.linear.y == FsbApprox(jac_vel.linear.y));
    REQUIRE(vel_expected.linear.z == FsbApprox(jac_vel.linear.z));
    // compare jacobian
    for (size_t row = 0; row < 6U; ++row)
    {
        for (size_t col = 0; col < panda_tree.get_num_dofs(); ++col)
        {
            REQUIRE(
                jac.j[fsb::jacobian_index(row, col)]
                == FsbApprox(jac_expected.j[fsb::jacobian_index(row, col)]));
        }
    }
}

TEST_CASE("Jacobian Panda Metrics" * doctest::description("[fsb_jacobian][fsb::calculate_jacobian]"))
{
    // Create the Panda robot's BodyTree
    size_t        ee_index = 0;
    fsb::BodyTree panda_tree = create_panda_body_tree(ee_index);
    // input position
    fsb::JointPva joint_pva = {};
    joint_pva.position.q[0] = 0.0;
    joint_pva.position.q[1] = -0.3;
    joint_pva.position.q[2] = 0.0;
    joint_pva.position.q[3] = -2.2;
    joint_pva.position.q[4] = 0.0;
    joint_pva.position.q[5] = 2.0;
    joint_pva.position.q[6] = 0.79;

    // Expected condition numbers
    const double expected_cond_num_linear = 7.64310978274591;
    const double expected_cond_num_angular = 4.125133262045351;

    // Compute FK
    fsb::CartesianPva base_pva = {};
    base_pva.pose = fsb::transform_identity();
    const fsb::ForwardKinematicsOption opt = fsb::ForwardKinematicsOption::POSE;
    fsb::BodyCartesianPva              cartesian_pva = {};
    fsb::forward_kinematics(panda_tree, joint_pva, base_pva, opt, cartesian_pva);
    // Compute jacobian
    fsb::Jacobian            jac = {};
    const fsb::JacobianError jac_err
        = fsb::calculate_jacobian(ee_index, panda_tree, cartesian_pva, jac);
    REQUIRE(jac_err == fsb::JacobianError::SUCCESS);
    // Compute jacobian metrics
    const fsb::JacobianMetrics jac_metrics
        = fsb::calculate_jacobian_metrics(jac, panda_tree.get_num_dofs());

    REQUIRE(jac_metrics.linear.condition_number == FsbApprox(expected_cond_num_linear));
    REQUIRE(jac_metrics.angular.condition_number == FsbApprox(expected_cond_num_angular));
}

TEST_SUITE_END();
