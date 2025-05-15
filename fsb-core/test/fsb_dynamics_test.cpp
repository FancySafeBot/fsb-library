
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_body_tree_sample.h"
#include "fsb_kinematics.h"
#include "fsb_dynamics.h"

TEST_SUITE_BEGIN("dynamics");

TEST_CASE("Inverse dynamics" * doctest::description("[fsb_dynamics][fsb::inverse_dynamics]"))
{
    // Inputs
    const fsb::Vec3 gravity = {0.0, 0.0, -9.81};
    // Base PVA
    const fsb::CartesianPva base_pva = {
        {
            {0.713252796614972, 0.110018106106709, 0.314124660380793, 0.616840467374075},
           {0.12, -0.34, 0.921}
        }, {},{}
    };
    // Parent-child transforms
    const fsb::Transform joint1_tr = {{0.57072141808226, 0.575121276132167, 0.0939451898978092, 0.578521289130613},{-0.872, 1.235, -0.02}};
    const fsb::Transform joint2_tr = {{0.466361491477014, -0.571547679819811, -0.124868616337094, 0.663511897115633}, {0.125, -0.2, 1}};
    const fsb::Transform joint3_tr = {{0.461283965309215, 0.607100248856612, 0.205771002489209, -0.613436782171915}, {-0.44, 0.2, -0.1}};
    // Joint PVA
    const fsb::JointPva joint_pva = {
        {{0.45, 1.73,  0.97}},
        {{-0.5, 0.71, -0.43}},
        {{1.5, 1.03, 0.62}}
    };
    // Mass properties
    fsb::MassProps body1_massprops = {
    0.8447, {0.01, 0.25, -0.17}, {1.21, 0.14, 0.2, 0.0, 0.0, 0.0}};
    fsb::MassProps body2_massprops = {
      0.3, {0.1, -0.0254, 0.05}, {0.78, 0.2, 0.99, 0.0, 0.0, 0.0}};
    fsb::MassProps body3_massprops = {
     0.1, {0.87, -0.11, 0.004}, {0.111, 1.54, 0.88, 0.0, 0.0, 0.0}};
    // Externally applied forces
    fsb::BodyForce external_force = {};

    // Expected
    // fsb::BodyForce expected_body_force = {};
    fsb::JointSpace expected_joint_torque = {};
    expected_joint_torque.qv[0] = -1.0903032895805924;
    expected_joint_torque.qv[1] = -2.9215345457188304;
    expected_joint_torque.qv[2] = 1.469521518393904;

    // Process
    // RPR model
    size_t last_body_index = 0U;
    fsb::BodyTree body_tree = body_tree_sample_rpr(
        joint1_tr, joint2_tr, joint3_tr,
        body1_massprops, body2_massprops, body3_massprops, last_body_index);
    body_tree.set_gravity(gravity);
    // forward kinematics
    fsb::BodyCartesianPva body_pva = {};
    const auto opt = fsb::ForwardKinematicsOption::POSE_VELOCITY_ACCELERATION;
    fsb::forward_kinematics(body_tree, joint_pva, base_pva, opt, body_pva);
    // inverse dynamics
    fsb::BodyForce actual_body_force = {};
    fsb::JointSpace actual_joint_torque = fsb::inverse_dynamics(
        body_tree, body_pva, external_force, actual_body_force);

    // Check
    CHECK(actual_joint_torque.qv[0] == FsbApprox(expected_joint_torque.qv[0]));
    CHECK(actual_joint_torque.qv[1] == FsbApprox(expected_joint_torque.qv[1]));
    CHECK(actual_joint_torque.qv[2] == FsbApprox(expected_joint_torque.qv[2]));
}

TEST_SUITE_END();
