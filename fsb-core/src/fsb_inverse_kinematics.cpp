#include <cstddef>
#include "fsb_inverse_kinematics.h"
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_configuration.h"
#include "fsb_jacobian.h"
#include "fsb_joint.h"
#include "fsb_kinematics.h"
#include "fsb_motion.h"
#include "fsb_types.h"
#include "fsb_linalg.h"

namespace fsb
{

static BodyCartesianPva optim_forward_kinematics(
    const BodyTree& body_tree, const Transform& base_pose, const JointSpacePosition& joint_position)
{
    BodyCartesianPva body_poses = {};
    // FK
    const JointPva     joint_pva = {joint_position, {}, {}};
    const CartesianPva base_pva = {base_pose, {}, {}};
    forward_kinematics(body_tree, joint_pva, base_pva, ForwardKinematicsOption::POSE, body_poses);
    return body_poses;
}

static Jacobian optim_jacobian(
    const BodyTree& body_tree, const size_t body_index, const BodyCartesianPva& body_poses)
{
    Jacobian jac = {};
    // Jacobian
    const JacobianError error = calculate_jacobian(body_index, body_tree, body_poses, jac);
    (void)error;
    return jac;
}

static real_t optim_pose_error(
    const MotionVector& cartesian_weights, const MotionVector& pose_error,
    MotionVector& weighted_pose_error)
{
    weighted_pose_error
        = {vector_multiply_elem(cartesian_weights.angular, pose_error.angular),
           vector_multiply_elem(cartesian_weights.linear, pose_error.linear)};
    const MotionVector squared_error
        = {vector_multiply_elem(pose_error.angular, weighted_pose_error.angular),
           vector_multiply_elem(pose_error.linear, weighted_pose_error.linear)};
    return squared_error.angular.x + squared_error.angular.y + squared_error.angular.z
           + squared_error.linear.x + squared_error.linear.y + squared_error.linear.z;
}

static FsbLinalgErrorType optim_solve_joint_matrix(
    const JointMatrix& joint_mat, const JointSpace& joint_vec, size_t dofs, JointSpace& result)
{
    // Create lapack input, work, and output variables
    const size_t nrhs = 1U;
    const size_t dim = dofs;
    const size_t work_len = MaxSize::dofs * MaxSize::dofs;
    double_t     work[MaxSize::dofs * MaxSize::dofs];
    const size_t iwork_len = MaxSize::dofs;
    int          iwork[MaxSize::dofs];
    // solve for x_vec
    return fsb_linalg_matrix_sqr_solve(
        joint_mat.j.data(), joint_vec.qv.data(), nrhs, dim, work_len, iwork_len, work, iwork, result.qv.data());
}

static InverseKinematicsResult optim_levenberg_marquardt(
    const BodyTree& body_tree, const OptimParameters& params,
    const JointSpacePosition& initial_config, size_t body_index, const Transform& target_pose,
    const Transform& base_pose, const size_t dofs)
{
    // initialize result
    InverseKinematicsResult result = {};
    result.info = InverseKinematicsInfo::SUCCESS;

    // Initialize satte vector
    JointSpace joint_offset = {};

    // First iteration
    // ===============

    // Update state: joint posiiton
    result.joint_position = initial_config;
    // Forward kinematics
    result.body_poses = optim_forward_kinematics(body_tree, base_pose, result.joint_position);
    // Jacobian
    result.jacobian = optim_jacobian(body_tree, body_index, result.body_poses);
    // Compute pose error
    result.computed_pose = result.body_poses.body[body_index].pose;
    result.error_pose = coord_transform_get_error(result.computed_pose, target_pose);
    // Weighted squared error
    MotionVector weighted_error = {};
    real_t obj_err = optim_pose_error(params.objective_weights, result.error_pose, weighted_error);

    // Start iterations
    // ================
    size_t iter = 1U;
    while ((result.info == InverseKinematicsInfo::SUCCESS) && (obj_err > params.objective_tol)
           && (iter < params.max_iterations))
    {
        // Compute Solution Increment
        // ==========================

        // Weighting matrix Wn = *E * EyeN + lambda * EyeN;
        JointSpace joint_weights = {};
        for (size_t index = 0U; index < dofs; ++index)
        {
            joint_weights.qv[index] = obj_err + params.damping_factor;
        }
        // Gradient vector g = J.transpose() * We * e;
        const JointSpace joint_gradient
            = jacobian_transpose_multiply(result.jacobian, weighted_error, dofs);
        // Joint increment matrix Mj = (J.transpose() * We * J + Wn)
        JointMatrix increment_matrix
            = jacobian_transpose_multiply_jacobian(result.jacobian, params.objective_weights, dofs);
        for (size_t index = 0U; index < dofs; ++index)
        {
            increment_matrix.j[joint_matrix_index(index, index, dofs)] += joint_weights.qv[index];
        }
        // Solve for increment dq = - Mj.inverse() * g
        const FsbLinalgErrorType solve_result
            = optim_solve_joint_matrix(increment_matrix, joint_gradient, dofs, joint_offset);
        if (solve_result != EFSB_LAPACK_ERROR_NONE)
        {
            // error
            result.info = InverseKinematicsInfo::SINGULAR_UPDATE_MATRIX;
        }
        else
        {
            // Iteration Result
            // ================

            // Update state: joint posiiton
            result.joint_position
                = joint_add_offset(body_tree, result.joint_position, joint_offset);
            // Forward kinematics
            result.body_poses
                = optim_forward_kinematics(body_tree, base_pose, result.joint_position);
            // Jacobian
            result.jacobian = optim_jacobian(body_tree, body_index, result.body_poses);
            // Compute pose error
            result.computed_pose = result.body_poses.body[body_index].pose;
            result.error_pose = coord_transform_get_error(result.computed_pose, target_pose);
            // Weighted squared error
            obj_err = optim_pose_error(params.objective_weights, result.error_pose, weighted_error);
            // ================

            // TODO: check relative changes in state vector and objective function

            // advance iteration
            iter += 1;
        }
    }

    if (iter == params.max_iterations)
    {
        result.info = InverseKinematicsInfo::MAXIMUM_EVALUATIONS_REACHED;
    }
    result.iterations = iter;
    return result;
}

InverseKinematicsResult compute_inverse_kinematics(
    const BodyTree& body_tree, const OptimParameters& params,
    const JointSpacePosition& initial_config, size_t body_index, const Transform& target_pose,
    const Transform& base_pose)
{
    // initialize result
    InverseKinematicsResult result = {};

    // number of dofs for selected body
    BodyTreeError err = BodyTreeError::SUCCESS;
    const size_t  dofs = body_tree.get_body_dofs(body_index, err);
    if (dofs == 0U || err != BodyTreeError::SUCCESS)
    {
        result.info = InverseKinematicsInfo::INVALID_INPUT;
    }
    else
    {
        result = optim_levenberg_marquardt(
            body_tree, params, initial_config, body_index, target_pose, base_pose, dofs);
    }
    return result;
}

FsbLinalgErrorType inverse_velocity_kinematics(const Jacobian& jacobian, const MotionVector& cart_velocity, size_t dofs, JointSpace& joint_velocity)
{
    if (dofs > MaxSize::dofs)
    {
        dofs = MaxSize::dofs;
    }

    const size_t nrhs = 1U; // number of right-hand sides
    // Solve the least squares problem J * qv = velocity
    // where J is the Jacobian and qv is the joint velocity vector.
    const double_t bvec[FSB_CART_SIZE] = {
        cart_velocity.angular.x, cart_velocity.angular.y, cart_velocity.angular.z,
        cart_velocity.linear.x, cart_velocity.linear.y, cart_velocity.linear.z
    };
    double_t work[512U] = {}; // workspace for least squares solve
    const size_t work_len = 512U; // length of workspace
    return fsb_linalg_leastsquares_solve(jacobian.j.data(), FSB_CART_SIZE, dofs, bvec, nrhs, work_len, work,
                                  joint_velocity.qv.data());
}

FsbLinalgErrorType inverse_acceleration_kinematics(
    const Jacobian& jacobian, const Jacobian& jacobian_derivative,
    const MotionVector& cart_acceleration, const JointSpace& joint_velocity, size_t dofs,
    JointSpace& joint_acceleration)
{

    const MotionVector cart_acc = jacobian_multiply(jacobian_derivative, joint_velocity, dofs);
    const MotionVector cart_motion = {
        vector_subtract(cart_acceleration.angular, cart_acc.angular),
        vector_subtract(cart_acceleration.linear, cart_acc.linear)
    };

    const double bvec[FSB_CART_SIZE]
        = {cart_motion.angular.x,
           cart_motion.angular.y,
           cart_motion.angular.z,
           cart_motion.linear.x,
           cart_motion.linear.y,
           cart_motion.linear.z};
    const size_t nrhs = 1U;
    double_t     work[512U] = {}; // workspace for least squares solve
    const size_t work_len = 512U; // length of workspace
    return fsb_linalg_leastsquares_solve(
        jacobian.j.data(),
        FSB_CART_SIZE,
        dofs,
        bvec,
        nrhs,
        work_len,
        work,
        joint_acceleration.qv.data());
}

} // namespace fsb
