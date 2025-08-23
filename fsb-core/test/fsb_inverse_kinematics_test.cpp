
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_kinematics.h"
#include "fsb_inverse_kinematics.h"
#include "fsb_body_tree_sample.h"
#include "fsb_rotation.h"

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

TEST_CASE("Velocity Inverse Kinematics Panda 7 DoF" * doctest::description("[fsb_inverse_kinematics]"))
{
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
    // target velocity
    const fsb::MotionVector ee_velocity = {
        {0.01, -0.1, 0.05},
        {0.1, 0.03, -0.4}
    };
    // target body
    const size_t ee_body_index = 8U;

    // expected
    const fsb::JointSpace expected_joint_velocity = {
        {
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0
        }
    };

    // get the number of degrees of freedom for the end-effector body
    fsb::BodyTreeError tree_err = {};
    const size_t ee_dofs = panda_tree.get_body_dofs(ee_body_index, tree_err);
    REQUIRE(tree_err == fsb::BodyTreeError::SUCCESS);

    // forward kinematics
    fsb::JointPva joint_pva = {initial_config, {}, {}};
    fsb::CartesianPva base_pva = {};
    fsb::BodyCartesianPva cartesian_pva = {};
    fsb::forward_kinematics(panda_tree, joint_pva, base_pva, fsb::ForwardKinematicsOption::POSE, cartesian_pva);

    // compute jacobian
    fsb::Jacobian jac = {};
    const fsb::JacobianError err = fsb::calculate_jacobian(ee_body_index, panda_tree, cartesian_pva, jac);
    REQUIRE(err == fsb::JacobianError::SUCCESS);

    // compute joint velocity from jacobian and target velocity
    fsb::JointSpace joint_velocity = {};
    const FsbLinalgErrorType linalg_err = fsb::inverse_velocity_kinematics(jac, ee_velocity, ee_dofs, joint_velocity);
    REQUIRE(linalg_err == FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE);

    // Check result
    REQUIRE(joint_velocity.qv[0] == FsbApprox(expected_joint_velocity.qv[0]));
    REQUIRE(joint_velocity.qv[1] == FsbApprox(expected_joint_velocity.qv[1]));
    REQUIRE(joint_velocity.qv[2] == FsbApprox(expected_joint_velocity.qv[2]));
    REQUIRE(joint_velocity.qv[3] == FsbApprox(expected_joint_velocity.qv[3]));
    REQUIRE(joint_velocity.qv[4] == FsbApprox(expected_joint_velocity.qv[4]));
    REQUIRE(joint_velocity.qv[5] == FsbApprox(expected_joint_velocity.qv[5]));
    REQUIRE(joint_velocity.qv[6] == FsbApprox(expected_joint_velocity.qv[6]));
}

//
// TEST_CASE("Acceleration Inverse Kinematics Panda 7 DoF" * doctest::description("[fsb_inverse_kinematics]"))
// {
//     const fsb::real_t pose_tolerance = 1.0e-4;
//     // Create the Panda robot's BodyTree
//     size_t        ee_index = 0;
//     fsb::BodyTree panda_tree = create_panda_body_tree(ee_index);
//
//     // input position and velocity vector
//     fsb::JointSpacePosition initial_config = {};
//     initial_config.q[0] = 1.0;
//     initial_config.q[1] = -0.32;
//     initial_config.q[2] = 0.08;
//     initial_config.q[3] = -2.15;
//     initial_config.q[4] = 0.04;
//     initial_config.q[5] = -2.0;
//     initial_config.q[6] = 0.78;
//     // target pose
//     const fsb::Transform target_pose = {
//     {0.9218430590013226, -0.3843272787743501, 0.04613060152655873, -0.019232393605190336},
//     {0.47372404011176217, 0.07, 0.5155132061520504}};
//     // target body
//     const size_t target_body_index = 8U;
//     // default optimization parameters
//     fsb::OptimParameters optim_params = fsb::default_optim_parameters();
//
//     // compute result
//     const fsb::InverseKinematicsResult result = fsb::compute_inverse_kinematics(panda_tree, optim_params, initial_config,
//         target_body_index, target_pose);
//
//     // Note: the manipulator is redundant so it does not have a single joint configuration as a solution.
//     // The computed pose is compared to the target pose.
//     REQUIRE(result.info == fsb::InverseKinematicsInfo::SUCCESS);
//     REQUIRE(result.computed_pose.rotation.qw == FsbApprox(target_pose.rotation.qw, pose_tolerance));
//     REQUIRE(result.computed_pose.rotation.qx == FsbApprox(target_pose.rotation.qx, pose_tolerance));
//     REQUIRE(result.computed_pose.rotation.qy == FsbApprox(target_pose.rotation.qy, pose_tolerance));
//     REQUIRE(result.computed_pose.rotation.qz == FsbApprox(target_pose.rotation.qz, pose_tolerance));
//     REQUIRE(result.computed_pose.translation.x == FsbApprox(target_pose.translation.x, pose_tolerance));
//     REQUIRE(result.computed_pose.translation.y == FsbApprox(target_pose.translation.y, pose_tolerance));
//     REQUIRE(result.computed_pose.translation.z == FsbApprox(target_pose.translation.z, pose_tolerance));
// }

TEST_SUITE_END();
