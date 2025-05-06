
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
 * @brief Jacobian multiply joint velocity
 *
 * @param jacobian Jacobian matrix
 * @param joint_motion Joint velocity
 * @param dofs Number of degrees of freedom (elements in joint velocity vector)
 * @return Cartesian velocity
 */
inline MotionVector jacobian_joint_multiply(
    const Jacobian& jacobian, const JointSpace& joint_motion, const size_t dofs = MaxSize::dofs)
{
    MotionVector result = {};
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        result.angular.x += jacobian.j[jacobian_index(0U, ind)] * joint_motion.qv[ind];
        result.angular.y += jacobian.j[jacobian_index(1U, ind)] * joint_motion.qv[ind];
        result.angular.z += jacobian.j[jacobian_index(2U, ind)] * joint_motion.qv[ind];
        result.linear.x += jacobian.j[jacobian_index(3U, ind)] * joint_motion.qv[ind];
        result.linear.y += jacobian.j[jacobian_index(4U, ind)] * joint_motion.qv[ind];
        result.linear.z += jacobian.j[jacobian_index(5U, ind)] * joint_motion.qv[ind];
    }
    return result;
}

/**
 * @brief Compute derivative of Jacobian matrix
 *
 * @param hessian Hessian matrix
 * @param joint_velocity Joint velocity vector
 * @param dofs Number of dofs (elements in joint velocity vector)
 * @return Jacobian derivative
 */
inline Jacobian jacobian_derivative(
    const Hessian& hessian, const JointSpace& joint_velocity, const size_t dofs = MaxSize::dofs)
{
    Jacobian result = {};
    for (size_t ind = 0U; ind < dofs; ++ind)
    {
        const MotionVector result_col = jacobian_joint_multiply(hessian.h[ind], joint_velocity);
        result.j[jacobian_index(0U, ind)] = result_col.angular.x;
        result.j[jacobian_index(1U, ind)] = result_col.angular.y;
        result.j[jacobian_index(2U, ind)] = result_col.angular.z;
        result.j[jacobian_index(3U, ind)] = result_col.linear.x;
        result.j[jacobian_index(4U, ind)] = result_col.linear.y;
        result.j[jacobian_index(5U, ind)] = result_col.linear.z;
    }
    return result;
}

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

MotionVector jacobian_multiply_joint_motion_vector(const Jacobian& jac, const JointSpace& joint_motion, size_t dofs = MaxSize::dofs);

/**
 * @}
 */

} // namespace fsb

#endif // FSB_JACOBIAN_H
