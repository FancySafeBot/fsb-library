
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_kinematics.h"
#include "fsb_inverse_kinematics.h"
#include "fsb_body_tree_sample.h"
#include "fsb_rotation.h"

TEST_SUITE_BEGIN("inverse_kinematics");

TEST_CASE("Inverse Kinematics Panda 7 DoF" * doctest::description("[fsb_inverse_kinematics]"))
{
    const fsb::Real pose_tolerance = 1.0e-4;
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
    fsb::OptimParameters optim_params = {};
    optim_params.max_iterations = 100U;
    optim_params.objective_tol = 1.0e-12;
    optim_params.state_tol = 1.0e-12;
    optim_params.damping_factor = 0.001;
    optim_params.objective_weights = {{1.0, 1.0, 1.0}, {1.0, 1.0, 1.0}};

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
        {0.1524867004188218, 0.07158684500011234, 0.2475327097076538},
        {-0.05354363789558364, 0.04979861410844365, 0.0009800931061712094}
    };
    // target body
    const size_t ee_body_index = 8U;

    // verify with alternative joint velocity
    const fsb::JointSpace verify_joint_velocity = {
        {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}
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
    const fsb::LinalgErrorType linalg_err = fsb::inverse_velocity_kinematics(jac, ee_velocity, ee_dofs, joint_velocity);
    REQUIRE(linalg_err == fsb::LinalgErrorType::ERROR_NONE);

    // Compute forward velocity kinematics to verify
    joint_pva.velocity = joint_velocity;
    fsb::forward_kinematics(panda_tree, joint_pva, base_pva, fsb::ForwardKinematicsOption::POSE_VELOCITY, cartesian_pva);

    REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.x == FsbApprox(ee_velocity.linear.x));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.y == FsbApprox(ee_velocity.linear.y));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.z == FsbApprox(ee_velocity.linear.z));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.x == FsbApprox(ee_velocity.angular.x));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.y == FsbApprox(ee_velocity.angular.y));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.z == FsbApprox(ee_velocity.angular.z));

    // Compute forward velocity kinematics with expected joint velocity
    joint_pva.velocity = verify_joint_velocity;
    fsb::forward_kinematics(panda_tree, joint_pva, base_pva, fsb::ForwardKinematicsOption::POSE_VELOCITY, cartesian_pva);

    REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.x == FsbApprox(ee_velocity.linear.x));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.y == FsbApprox(ee_velocity.linear.y));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.z == FsbApprox(ee_velocity.linear.z));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.x == FsbApprox(ee_velocity.angular.x));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.y == FsbApprox(ee_velocity.angular.y));
    REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.z == FsbApprox(ee_velocity.angular.z));

}

// TEST_CASE("Acceleration Inverse Kinematics Panda 7 DoF" * doctest::description("[fsb_inverse_kinematics]"))
// {
//     // Create the Panda robot's BodyTree
//     size_t        ee_index = 0;
//     fsb::BodyTree panda_tree = create_panda_body_tree(ee_index);

//     // input position and velocity vector
//     fsb::JointSpacePosition initial_config = {};
//     initial_config.q[0] = 1.0;
//     initial_config.q[1] = -0.32;
//     initial_config.q[2] = 0.08;
//     initial_config.q[3] = -2.15;
//     initial_config.q[4] = 0.04;
//     initial_config.q[5] = -2.0;
//     initial_config.q[6] = 0.78;
//     // target velocity
//     const fsb::MotionVector ee_velocity = {
//         {0.1524867004188218, 0.07158684500011234, 0.2475327097076538},
//         {-0.05354363789558364, 0.04979861410844365, 0.0009800931061712094}
//     };
//     const fsb::MotionVector ee_acceleration = {
//         {0.07181461149754069, 0.07105335307534937, 0.12795685907522877},
//         {-0.03432487144790067, 0.014343111642331922, -0.005645573256824818}
//     };
//     // target body
//     const size_t ee_body_index = 8U;

// // std::vector<double> q_pos = {1.0, -0.32, 0.08, -2.15, 0.04, -2.0, 0.78};
// // std::vector<double> q_vel = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
// // std::vector<double> q_acc = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
// // Eigen::Vector3d ee_pos << 0.17135057568505976, 0.32983697206672097, 0.7783051072071709;
// // Eigen::Vector3d ee_quat << 0.47787245515993504, -0.2938045146353132, -0.1669411141557633, -0.8108313561917517;
// // Eigen::Vector3d ee_vel_linear << -0.05354363789558364, 0.04979861410844365, 0.0009800931061712094;
// // Eigen::Vector3d ee_vel_angular << 0.1524867004188218, 0.07158684500011234, 0.2475327097076538;
// // Eigen::Vector3d ee_acc_linear << -0.03432487144790067, 0.014343111642331922, -0.005645573256824818;
// // Eigen::Vector3d ee_acc_angular << 0.07181461149754069, 0.07105335307534937, 0.12795685907522877;

//     // verify with alternative joint velocity
//     const fsb::JointSpace verify_joint_velocity = {
//         {0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1}
//     };
//     const fsb::JointSpace verify_joint_acceleration = {
//         {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05}
//     };

//     // get the number of degrees of freedom for the end-effector body
//     fsb::BodyTreeError tree_err = {};
//     const size_t ee_dofs = panda_tree.get_body_dofs(ee_body_index, tree_err);
//     REQUIRE(tree_err == fsb::BodyTreeError::SUCCESS);

//     // forward kinematics
//     fsb::JointPva joint_pva = {initial_config, {}, {}};
//     fsb::CartesianPva base_pva = {};
//     fsb::BodyCartesianPva cartesian_pva = {};
//     fsb::forward_kinematics(panda_tree, joint_pva, base_pva, fsb::ForwardKinematicsOption::POSE, cartesian_pva);

//     // compute jacobian
//     fsb::Jacobian jac = {};
//     const fsb::JacobianError err = fsb::calculate_jacobian(ee_body_index, panda_tree, cartesian_pva, jac);
//     REQUIRE(err == fsb::JacobianError::SUCCESS);

//     // compute joint velocity from jacobian and target velocity
//     fsb::JointSpace joint_velocity = {};
//     const fsb::LinalgErrorType linalg_err = fsb::inverse_velocity_kinematics(jac, ee_velocity, ee_dofs, joint_velocity);
//     REQUIRE(linalg_err == fsb::LinalgErrorType::ERROR_NONE);

//     fsb::inverse_acceleration_kinematics(jac, ee_velocity, ee_acceleration, ee_dofs, joint_velocity, joint_pva.velocity, joint_pva.acceleration);

//     // Compute forward velocity kinematics to verify
//     joint_pva.velocity = joint_velocity;
//     fsb::forward_kinematics(panda_tree, joint_pva, base_pva, fsb::ForwardKinematicsOption::POSE_VELOCITY, cartesian_pva);

//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.x == FsbApprox(ee_velocity.linear.x));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.y == FsbApprox(ee_velocity.linear.y));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.z == FsbApprox(ee_velocity.linear.z));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.x == FsbApprox(ee_velocity.angular.x));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.y == FsbApprox(ee_velocity.angular.y));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.z == FsbApprox(ee_velocity.angular.z));

//     // Compute forward velocity kinematics with expected joint velocity
//     joint_pva.velocity = verify_joint_velocity;
//     fsb::forward_kinematics(panda_tree, joint_pva, base_pva, fsb::ForwardKinematicsOption::POSE_VELOCITY, cartesian_pva);

//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.x == FsbApprox(ee_velocity.linear.x));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.y == FsbApprox(ee_velocity.linear.y));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.linear.z == FsbApprox(ee_velocity.linear.z));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.x == FsbApprox(ee_velocity.angular.x));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.y == FsbApprox(ee_velocity.angular.y));
//     REQUIRE(cartesian_pva.body[ee_body_index].velocity.angular.z == FsbApprox(ee_velocity.angular.z));

// }

TEST_SUITE_END();
