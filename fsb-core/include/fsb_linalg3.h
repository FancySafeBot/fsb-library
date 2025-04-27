
#pragma once

#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup LinearAlgebra3 Linear Algebra for 3x3 Matrices
 * @brief Linear algebra for 3x3 matrix operations.
 * @{
 */

/**
 * @brief Compute eigenvalues of a 3x3 symmetric positive definite matrix.
 *
 * @param[in] mat Input symmetric matrix
 * @param[out] eigenvalues Eigenvalues of input matrix
 * @return true if matrix is positive definite. Eigenvalues computed successfully.
 * @return false if matrix is not positive definite. Eigenvalues are not set.
 */
bool mat3_posdef_symmetric_eigenvalues(const Mat3Sym& mat, Vec3& eigenvalues);

/**
 * @brief Compute eigenvalues and eigenvectors of a 3x3 symmetric positive definite matrix.
 *
 * Eigen values are ordered with smallest value at x component of output vector, largest at z and
 * middle value at y
 *
 * @param[in] mat_in Input matrix
 * @param[out] eigenvalues Eigenvalues of input matrix
 * @param[out] eig_vec0 Eigenvector of input matrix corresponding to eigenvalue x
 * @param[out] eig_vec1 Eigenvector of input matrix corresponding to eigenvalue y
 * @param[out] eig_vec2 Eigenvector of input matrix corresponding to eigenvalue z
 * @return true if matrix is positive definite. Eigen values and vectors computed successfully.
 * @return false if matrix is not positive definite. Eigenvalues and vectors are not set.
 */
bool mat3_posdef_symmetric_eigenvectors(
    const Mat3Sym& mat_in, Vec3& eigenvalues, Vec3& eig_vec0, Vec3& eig_vec1, Vec3& eig_vec2);

/**
 * @}
 */

} // namespace fsb
