
#pragma once

#include "fsb_quaternion.h"
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup TopicRotation Rotation matrices and conversions
 * @brief Conversions and operations for rotations in SO(3)
 * @{
 */

/**
 * @brief Skew-symmetric matrix from vector
 *
 * \f$ [v]^{\times} = \begin{bmatrix} 0 & -v_z & v_y \\ v_z & 0 & -v_x \\ -v_y & v_x & 0
 * \end{bmatrix} \f$
 *
 * @param vec Vector
 * @return Skew-symmetric matrix
 */
Mat3 skew_symmetric(const Vec3& vec);

/**
 * @brief Convert unit quaternion to intrinsic ZYX Euler-angles
 *
 * @param quat Quaternion
 * @return Euler angles in radians
 */
Vec3 quat_to_ezyx(const Quaternion& quat);

/**
 * @brief Convert intrinsic ZYX Euler-angles to unit quaternion
 *
 * @param ezyx Euler angles in radians
 * @return Quaternion
 */
Quaternion ezyx_to_quat(const Vec3& ezyx);

/**
 * @brief Convert unit quaternion to rotation matrix
 *
 * @param quat Quaternion
 * @return Rotation matrix
 */
Mat3 quat_to_rot(const Quaternion& quat);

/**
 * @brief Convert rotation matrix to unit quaternion
 *
 * @param rot Rotation matrix
 * @return Quaternion
 */
Quaternion rot_to_quat(const Mat3& rot);

/**
 * @brief Convert unit quaternion to rotation vector
 *
 * @param quat Quaternion
 * @return Rotation vector
 */
Vec3 quat_to_rotvec(const Quaternion& quat);

/**
 * @brief Convert rotation vector to unit quaternion
 *
 * @param rotvec Rotation vector
 * @return Quaternion
 */
Quaternion rotvec_to_quat(const Vec3& rotvec);

/**
 * @brief Identity rotation matrix
 *
 * \f$ \begin{bmatrix} 1 & 0 & 0 \\ 0 & 1 & 0 \\ 0 & 0 & 1 \end{bmatrix} \f$
 *
 * @return Identity rotation matrix
 */
Mat3 rot_identity();

/**
 * @brief Rotation matrix about x-axis
 *
 * @param rx Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rot_rx(real_t rx);

/**
 * @brief Rotation matrix about y-axis
 *
 * @param ry Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rot_ry(real_t ry);

/**
 * @brief Rotation matrix about z-axis
 *
 * @param rz Rotation angle in radians
 * @return Rotation matrix
 */
Mat3 rot_rz(real_t rz);

/**
 * @brief Matrix multiplication with rotation matrix
 *
 * @param rot Rotation matrix
 * @param vec Vector
 * @return Rotated vector
 */
Vec3 rotate_mat3(const Mat3& rot, const Vec3& vec);

/**
 * @brief Transposed matrix multiplication with rotation matrix
 *
 * @param rot Rotation matrix
 * @param vec Vector
 * @return Rotated vector with transpose of rotation matrix
 */
Vec3 rotate_mat3_transpose(const Mat3& rot, const Vec3& vec);

/**
 * @}
 */

} // namespace fsb
