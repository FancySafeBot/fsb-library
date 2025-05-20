
#ifndef FSB_QUATERNION_H
#define FSB_QUATERNION_H

#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup TopicQuaternion Unit Quaternions
 * @brief Rotations using quaternions
 * @{
 */

/**
 * @brief Unit quaternion
 *
 * \f[
 *
 * \f]
 */
struct Quaternion
{
    /**
     * @brief Scalar component
     */
    real_t qw;
    /**
     * @brief X component
     */
    real_t qx;
    /**
     * @brief Y component
     */
    real_t qy;
    /**
     * @brief Z component
     */
    real_t qz;
};

/**
 * @brief Multiply two quaternions
 *
 * Returns quaternion multiplication:
 * \f[
 * q = q_a q_b
 * \f]
 *
 * @param q_a Quaterion a
 * @param q_b Quaternion b
 * @return Quaternion product of a and b
 */
Quaternion quat_multiply(const Quaternion& q_a, const Quaternion& q_b);

/**
 * @brief Rotate vector by quaternion
 *
 * @param q_in Quaternion
 * @param p_in Point to rotate
 * @return Rotated point
 */
Vec3 quat_rotate_vector(const Quaternion& q_in, const Vec3& p_in);

/**
 * @brief Conjugate of quaternion
 *
 * Returns conjugate:
 * \f$ q^* = [q_w, -q_x, -q_y, -q_z] \f$
 *
 * @param q_in Quaternion
 * @return Conjugate of quaternion
 */
Quaternion quat_conjugate(const Quaternion& q_in);

/**
 * @brief 2-norm of quaternion
 *
 * \f$ \| q \| = \sqrt{q_w^2 + q_x^2 + q_y^2 + q_z^2} \f$
 *
 * @param q_in Quaternion
 * @return Norm of quaternion
 */
real_t quat_norm(const Quaternion& q_in);

/**
 * @brief Normalize quaternion
 *
 * @param q_inout Quaternion to normalize on input, normalized on output
 */
void quat_normalize(Quaternion& q_inout);

/**
 * @brief Logarithm of unit quaternion
 *
 * @param q_in Unit quaternion
 * @return Logarithm of unit quaternion
 */
Vec3 quat_log(const Quaternion& q_in);

/**
 * @brief Exponential of quaternion vector
 *
 * @param v_in Quaternion vector with assumed scalar component of zero
 * @return Unit quaternion
 */
Quaternion quat_exp(const Vec3& v_in);

/**
 * @brief Add body-fixed rotation vector to quaternion
 *
 * \f$ q \boxplus \phi = q \exp\left(\tfrac{1}{2}\phi \right) \f$
 *
 * @param quat Quaternion to rotate
 * @param phi_delta Rotation vector in radians
 * @return Rotated quaternion
 */
Quaternion quat_boxplus(const Quaternion& quat, const Vec3& phi_delta);

/**
 * @brief Get body-fixed rotation vector from quaternion difference
 *
 * \f$ q_a \boxminus q_b = 2 \log(q_b^{-1} q_a) \f$
 *
 * @param q_a Orientation a
 * @param q_b Orietnation b
 * @return Body-fixed rotation vector from b to a in radians
 */
Vec3 quat_boxminus(const Quaternion& q_a, const Quaternion& q_b);

/**
 * @brief Add space frame rotation vector to quaternion
 *
 * \f$ q \oplus \phi = \exp\left(\tfrac{1}{2}\phi \right) q \f$
 *
 * @param quat Initial orientation
 * @param phi_delta Axis-angle rotation vector
 * @return Final orientation
 */
Quaternion quat_circplus(const Quaternion& quat, const Vec3& phi_delta);

/**
 * @brief Get space frame rotation vector from quaternion difference
 *
 * \f$ q_a \ominus q_b = 2 \log(q_a q_b^{-1}) \f$
 *
 * @param q_a Orientation a
 * @param q_b Orietnation b
 * @return Space frame rotation vector from b to a in radians
 */
Vec3 quat_circminus(const Quaternion& q_a, const Quaternion& q_b);

/**
 * @brief Identity quaternion
 *
 * \f$ q = [1, 0, 0, 0] \f$
 *
 * @return Identity quaternion
 */
Quaternion quat_identity();

/**
 * @brief Rotation about x-axis
 *
 * @param rx Rotation angle in radians
 * @return Rotation unit quaternion
 */
Quaternion quat_rx(real_t rx);

/**
 * @brief Rotation about y-axis
 *
 * @param ry Rotation angle in radians
 * @return Rotation unit quaternion
 */
Quaternion quat_ry(real_t ry);

/**
 * @brief Rotation about z-axis
 *
 * @param rz Rotation angle in radians
 * @return Rotation unit quaternion
 */
Quaternion quat_rz(real_t rz);

/**
 * @}
 */

} // namespace fsb

#endif
