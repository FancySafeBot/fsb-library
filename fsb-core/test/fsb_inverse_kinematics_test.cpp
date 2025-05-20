
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_kinematics.h"
#include "fsb_inverse_kinematics.h"
#include "fsb_body_tree_sample.h"
#include "fsb_rotation.h"

#include <iostream>

TEST_SUITE_BEGIN("inverse_kinematics");

TEST_CASE("Inverse Kinematics Panda 7 DoF" * doctest::description("[fsb_inverse_kinematics]"))
{
    const fsb::real_t pose_tolerance = 1.0e-4;
    // Create the Panda robot's BodyTree
    size_t        ee_index = 0;
    fsb::BodyTree panda_tree = create_panda_body_tree(ee_index);

    // input position and velocity vector
    fsb::JointSpacePosition initial_config = {};
    initial_config.q[0] = 1.0;
    initial_config.q[1] = -0.32;
    initial_config.q[2] = 0.08;
    initial_config.q[3] = -2.15;
    initial_config.q[4] = 0.04;
    initial_config.q[5] = -2.0;
    initial_config.q[6] = 0.78;
    // target pose
    const fsb::Transform target_pose = {
    {0.9218430590013226, -0.3843272787743501, 0.04613060152655873, -0.019232393605190336},
    {0.47372404011176217, 0.07, 0.5155132061520504}};
    // target body
    const size_t target_body_index = 8U;
    // default optimization parameters
    fsb::OptimParameters optim_params = fsb::default_optim_parameters();

    // compute result
    const fsb::InverseKinematicsResult result = fsb::compute_inverse_kinematics(panda_tree, optim_params, initial_config,
        target_body_index, target_pose);
    std::cout << "Num Iterations: " << static_cast<int>(result.iterations) << std::endl;

    // Note: the manipulator is redundant so it does not have a single joint configuration as a solution.
    // The computed pose is compared to the target pose.
    REQUIRE(result.info == fsb::InverseKinematicsInfo::SUCCESS);
    REQUIRE(result.computed_pose.rotation.qw == FsbApprox(target_pose.rotation.qw, pose_tolerance));
    REQUIRE(result.computed_pose.rotation.qx == FsbApprox(target_pose.rotation.qx, pose_tolerance));
    REQUIRE(result.computed_pose.rotation.qy == FsbApprox(target_pose.rotation.qy, pose_tolerance));
    REQUIRE(result.computed_pose.rotation.qz == FsbApprox(target_pose.rotation.qz, pose_tolerance));
    REQUIRE(result.computed_pose.translation.x == FsbApprox(target_pose.translation.x, pose_tolerance));
    REQUIRE(result.computed_pose.translation.y == FsbApprox(target_pose.translation.y, pose_tolerance));
    REQUIRE(result.computed_pose.translation.z == FsbApprox(target_pose.translation.z, pose_tolerance));
}

TEST_SUITE_END();
