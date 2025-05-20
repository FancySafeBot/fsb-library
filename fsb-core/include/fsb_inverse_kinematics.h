//
// Created by Kyle Chisholm on 2025-05-16.
//

#ifndef FSB_INVERSE_KINEMATICS_H
#define FSB_INVERSE_KINEMATICS_H

#include <array>
#include <cstddef>
#include <cstdint>
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_configuration.h"
#include "fsb_motion.h"
#include "fsb_joint.h"
#include "fsb_types.h"
#include "fsb_jacobian.h"

namespace fsb
{

/**
 * @defgroup InverseKinematics Inverse Kinematics
 * @brief Inverse kinematics
 *
 * @{
 */

struct OptimParameters
{
    size_t max_iterations; ///< Maximum number of iterations
    real_t objective_tol; ///< Objective function error tolerance for convergence
    real_t state_tol; ///< State vector error tolerance for convergence
    real_t damping_factor; ///< Damping factor for Levenberg-Marquardt algorithm
    MotionVector objective_weights; ///< Cartesian weights for objective function
};

enum class InverseKinematicsInfo : uint8_t
{
    INVALID_INPUT = 0,
    SUCCESS = 1, ///< objective function converged
    WITHIN_FTOL = 2, ///< both actual and predicted relative reductions in the sum of squares are at most ftol
    WITHIN_XTOL = 3, ///< relative error between two consecutive iterates is at most xtol
    WITHIN_FTOL_XTOL = 4, ///< conditions for WITHIN_FTOL and WITHIN_XTOL both hold
    MAXIMUM_EVALUATIONS_REACHED = 5, ///< number of calls to fcn with iflag = 1 has reached maxfev
    SINGULAR_UPDATE_MATRIX = 6 ///< singular matrix encountered
};

struct InverseKinematicsResult
{
    InverseKinematicsInfo info; ///< Convergence information
    JointSpacePosition joint_position; ///< Joint position
    BodyCartesianPva body_poses; ///< Body poses
    Jacobian jacobian; ///< Jacobian matrix
    Transform computed_pose; ///< Computed end-effector pose
    MotionVector error_pose; ///< Error between computed and desired pose
    size_t iterations; ///< Number of iterations
};

inline OptimParameters default_optim_parameters()
{
    return {
        100U, // max iterations
        1.0e-12, // objective tolerance
        1.0e-12, // f tolerance
        0.001,
        {
            {1.0, 1.0, 1.0},
            {1.0, 1.0, 1.0}
        }
    };
}

/**
 * @brief Compute inverse kinematics for a given end-effector pose
 *
 * @param body_tree Body tree with link definitions
 * @param params Optimization parameters
 * @param initial_config Initial joint configuration
 * @param body_index Index of body to compute inverse kinematics for
 * @param target_pose Desired body target pose
 * @param base_pose Base pose (default is identity transform)
 * @return Result of inverse kinematics computation
 */
InverseKinematicsResult compute_inverse_kinematics(
    const BodyTree& body_tree, const OptimParameters& params, const JointSpacePosition& initial_config, size_t body_index,
    const Transform& target_pose, const Transform& base_pose = transform_identity());

/**
 * @}
 */

}

#endif //FSB_INVERSE_KINEMATICS_H
