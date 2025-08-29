
#ifndef FSB_KINEMATICS_REDUNDANCY_H
#define FSB_KINEMATICS_REDUNDANCY_H

#include "fsb_joint.h"
#include "fsb_linalg.h"
#include "fsb_jacobian.h"

namespace fsb
{

/**
 * @defgroup KinematicsRedundancy Redundant Robotics Kinematics
 * @brief Redundancy resolution with joint limit avoidance and singularity avoidance.
 *
 * @{
 */

/**
 * @brief Computes the pseudoinverse of a Jacobian matrix
 *
 * Calculates the Moore-Penrose pseudoinverse \f$ \mathbf{J}^{+} \f$ of the provided Jacobian matrix
 * using SVD decomposition. The pseudoinverse is necessary for handling redundant or
 * underconstrained manipulators.
 *
 * For a Jacobian matrix \f$ \mathbf{J} \f$, the pseudoinverse \f$ \mathbf{J}^{+} \f$ satisfies:
 * \f[
 *    \mathbf{J}^{+} = \mathbf{V} \mathbf{\Sigma}^{+} \mathbf{U}^T
 * \f]
 * where \f$ \mathbf{J} = \mathbf{U} \mathbf{\Sigma} \mathbf{V}^T \f$ is the SVD decomposition.
 *
 * @param[in]  jacobian         Input Jacobian matrix \f$ \mathbf{J} \f$
 * @param[out] inverse_jacobian Output pseudoinverse of the Jacobian \f$ \mathbf{J}^{+} \f$
 * @param[in]  dofs             Number of degrees of freedom (columns in Jacobian)
 * @return                      Linear algebra error code
 */
FsbLinalgErrorType jacobian_pseudoinverse(const Jacobian& jacobian, Jacobian& inverse_jacobian, size_t dofs);

/**
 * @brief Computes nullspace motion using the Jacobian and its pseudoinverse
 *
 * Projects joint motion into the nullspace, resulting in motion that doesn't affect the end-effector pose.
 * This is useful for secondary objectives like joint limit avoidance while maintaining a primary task.
 *
 * The nullspace projection is computed as:
 * \f[
 *    \dot{\mathbf{q}}_{null} = (\mathbf{I} - \mathbf{J}^{+} \mathbf{J}) \dot{\mathbf{q}}
 * \f]
 *
 * Where:
 * - \f$ \dot{\mathbf{q}}_{null} \f$ is the nullspace component of joint motion
 * - \f$ \mathbf{I} \f$ is the identity matrix
 * - \f$ \mathbf{J}^{+} \f$ is the pseudoinverse of the Jacobian
 * - \f$ \mathbf{J} \f$ is the Jacobian matrix
 * - \f$ \dot{\mathbf{q}} \f$ is the input joint motion vector
 *
 * @param[in] jacobian          Input Jacobian matrix \f$ \mathbf{J} \f$
 * @param[in] inverse_jacobian  Pseudoinverse of the Jacobian matrix \f$ \mathbf{J}^{+} \f$
 * @param[in] joint_motion      Input joint motion vector \f$ \dot{\mathbf{q}} \f$ to project into nullspace
 * @param[in] dofs              Number of degrees of freedom (elements in joint velocity vector)
 * @return                      Joint motion vector in the nullspace \f$ \dot{\mathbf{q}}_{null} \f$
 */
JointSpace compute_nullspace_motion(const Jacobian& jacobian, const Jacobian& inverse_jacobian, const JointSpace& joint_motion, size_t dofs);

/**
 *
 * @brief Computes nullspace motion by least squares solution without explicit pseudoinverse
 *
 * @param[in] jacobian          Input Jacobian matrix \f$ \mathbf{J} \f$
 * @param[in] joint_motion      Input joint motion vector \f$ \dot{\mathbf{q}} \f$ to project into nullspace
 * @param[in] dofs              Number of degrees of freedom (elements in joint velocity vector)
 * @return                      Joint motion vector in the nullspace \f$ \dot{\mathbf{q}}_{null} \f$
 */
JointSpace compute_nullspace_motion(const Jacobian& jacobian, const JointSpace& joint_motion, size_t dofs);

/**
 * @}
 */

} // namespace fsb

#endif
