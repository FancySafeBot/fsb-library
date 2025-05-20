
#ifndef FSB_JACOBIAN_H
#define FSB_JACOBIAN_H

#include <array>
#include <cstddef>
#include <cstdint>
#include "fsb_body.h"
#include "fsb_body_tree.h"
#include "fsb_configuration.h"
#include "fsb_joint.h"
#include "fsb_motion.h"
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup Jacobian Jacobian Kinematics
 * @brief Jacobian matrix for robotics
 * @{
 */

/**
 * @brief Jacobian matrix
 */
struct Jacobian
{
    /**
     * @brief Jacobian matrix data array
     */
    std::array<real_t, 6U * MaxSize::dofs> j;
};

/**
 * @brief Square matrix of size nxn for n joints
 */
struct JointMatrix
{
    /**
     * @brief Square matrix data array
     */
    std::array<real_t, MaxSize::dofs * MaxSize::dofs> j;
};

/**
 * @brief Hessian tensor
 */
struct Hessian
{
    /**
     * @brief Hessian tensor data "pages"
     */
    std::array<Jacobian, MaxSize::dofs> h;
};

/**
 * @brief Error values for Jacobian calculations
 */
enum class JacobianError : uint8_t
{
    /**
     * @brief No error
     */
    SUCCESS,
    /**
     * @brief Body index not found in tree
     */
    BODY_NOT_IN_TREE
};

/**
 * @brief Helper function for indexing jacobian matrix
 *
 * Matrix is ordered column-major with 6 rows
 *
 * @param row Row index
 * @param col Column index
 * @return Jacobian index
 */
inline size_t jacobian_index(const size_t row, const size_t col)
{
    return 6U * col + row;
}

/**
 * @brief Helper function for indexing joint matrix
 *
 * Matrix is ordered column-major with n rows for n joints
 *
 * @param row Row index
 * @param col Column index
 * @param dofs Number of degrees of freedom (elements in joint velocity vector)
 * @return Array index for joint matrix
 */
inline size_t joint_matrix_index(const size_t row, const size_t col, const size_t dofs)
{
    return dofs * col + row;
}

/**
 * @brief Jacobian multiply joint velocity
 *
 * @param jacobian Jacobian matrix
 * @param joint_motion Joint velocity
 * @param dofs Number of degrees of freedom (elements in joint velocity vector)
 * @return Cartesian velocity
 */
MotionVector jacobian_multiply(
    const Jacobian& jacobian, const JointSpace& joint_motion, size_t dofs = MaxSize::dofs);

/**
 * @brief Jacobian transpose multiply cartesian motion
 *
 * @param jacobian Jacobian matrix
 * @param cartesian_motion Cartesian motion
 * @param dofs Number of degrees of freedom (elements in joint motion vector)
 * @return Joint motion
 */
JointSpace jacobian_transpose_multiply(
    const Jacobian& jacobian, const MotionVector& cartesian_motion, size_t dofs = MaxSize::dofs);

/**
 * @brief Jacobian transpose multiply jacobian
 *
 * \f$ J^T \cdot \text{diag}(w) J \f$
 *
 * @param jacobian Jacobian matrix
 * @param cartesian_weights Weighted cartesian motion
 * @param dofs
 * @return
 */
JointMatrix jacobian_transpose_multiply_jacobian(const Jacobian& jacobian, const MotionVector& cartesian_weights, size_t dofs = MaxSize::dofs);

/**
 * @brief Compute derivative of Jacobian matrix
 *
 * @param hessian Hessian matrix
 * @param joint_velocity Joint velocity vector
 * @param dofs Number of dofs (elements in joint velocity vector)
 * @return Jacobian derivative
 */
Jacobian jacobian_derivative(
    const Hessian& hessian, const JointSpace& joint_velocity, size_t dofs = MaxSize::dofs);

/**
 * @brief Determine Jacobian matrix for a single body in tree
 *
 * Cartesian pose for all bodies should be determined by forward kinematics
 * prior to calling this function.
 *
 * @param[in] body_index Body index of Jacobian
 * @param[in] body_tree Body tree with body and joint data
 * @param[in] cartesian_pva Cartesian pose, velocity and acceleration data for all bodies
 * @param[out] jacobian Jacobian matrix for body
 * @return Jacobian error code
 */
JacobianError calculate_jacobian(
    size_t body_index, const BodyTree& body_tree, const BodyCartesianPva& cartesian_pva,
    Jacobian& jacobian);

/**
 * @}
 */

} // namespace fsb

#endif // FSB_JACOBIAN_H
