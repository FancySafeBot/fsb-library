
#ifndef FSB_MOTION_H
#define FSB_MOTION_H

#include "fsb_quaternion.h"
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup MotionCoord Coordinate Transforms and Motion Vectors
 * @brief Coordinate transformations and velocity and acceleration propagation.
 *
 * @{
 */

/**
 * @brief Coordinate transform
 */
struct Transform
{
    /**
     * @brief Rotation quaternion
     */
    Quaternion rotation;
    /**
     * @brief Translation vector
     */
    Vec3 translation;
};

/**
 * @brief Motion vector
 */
struct MotionVector
{
    /**
     * @brief Angular motion vector
     */
    Vec3 angular;
    /**
     * @brief Linear motion vector
     *
     */
    Vec3 linear;
};

/**
 * @brief Cartesian pose, velocity and acceleration
 */
struct CartesianPva
{
    /**
     * @brief Cartesian pose
     */
    Transform pose;
    /**
     * @brief Cartesian velocity
     */
    MotionVector velocity;
    /**
     * @brief Cartesian acceleration
     */
    MotionVector acceleration;
};

/**
 * @brief Vector addition
 *
 * \f$ \mathbf{v}_a + \mathbf{v}_b \f$
 *
 * @param v_a Vector a
 * @param v_b Vector b
 * @return Vector addition of vector a and b
 */
Vec3 vector_add(const Vec3& v_a, const Vec3& v_b);

/**
 * @brief Vector subtraction
 *
 * \f$ \mathbf{v}_a - \mathbf{v}_b \f$
 *
 * @param v_a Vector a
 * @param v_b Vector b
 * @return Vector subtraction of b from a
 */
Vec3 vector_subtract(const Vec3& v_a, const Vec3& v_b);

/**
 * @brief Scale vector
 *
 * \f$ s \mathbf{v} \f$
 *
 * @param scalar Scalar value
 * @param vec Vector to scale
 * @return Scaled Vector
 */
Vec3 vector_scale(real_t scalar, const Vec3& vec);

/**
 * @brief Vector cross product
 *
 * \f$ \mathbf{v}_a \times \mathbf{v}_b \f$
 *
 * @param v_a Vector a
 * @param v_b Vector b
 * @return Cross product of a and b
 */
Vec3 vector_cross(const Vec3& v_a, const Vec3& v_b);

/**
 * @brief Vector dot product
 *
 * \f$ \mathbf{v}_a \cdot \mathbf{v}_b \f$
 *
 * @param v_a Vector a
 * @param v_b Vector b
 * @return Dot product of a and b
 */
real_t vector_dot(const Vec3& v_a, const Vec3& v_b);

/**
 * @brief Vector norm
 *
 * \f$ \| \mathbf{v} \| = \sqrt{v_x^2 + v_y^2 + v_z^2} \f$
 *
 * @param vec Vector input
 * @return 2-norm of vector
 */
real_t vector_norm(const Vec3& vec);

/**
 * @brief Identity transform
 *
 * Rotation is unit quaternion with zero x, y, and z components
 * Zero vector translation
 *
 * @return Identity transform
 */
Transform transform_identity();

/**
 * @brief Get inverse of transform
 *
 * @param transf Input transform
 * @return Inverse of transform
 */
Transform transform_inverse(const Transform& transf);

/**
 * @brief Apply coordinate transform
 *
 *  The resulting orientation quaternion is
 *  \f[
 *  q = q_A q_B
 *  \f]
 *  and the position is
 *  \f[
 *  p = p_A + C(q_A) p_B
 *  \f]
 *
 * @param transf_a Transform A
 * @param transf_b Transform B
 * @return Transform A applied to B
 */
extern Transform coord_transform(const Transform& transf_a, const Transform& transf_b);

/**
 * @brief Apply coordinate transform to position vector
 *
 * @param transf Coordinate transform
 * @param pos Position vector
 * @return Transformed position vector
 */
Vec3 coord_transform_position(const Transform& transf, const Vec3& pos);

/**
 * @brief Apply motion vector offset to transform in space frame
 *
 * @param offset Offset motion vector in space frame
 * @param transf Coordinate transform applied after motion offset
 * @return Final transform including offset applied in space frame
 */
Transform coord_transform_apply_space_offset(const MotionVector& offset, const Transform& transf);

/**
 * @brief Get motion vector offset between two coordinate transforms in space frame
 *
 * @param transf_post Coordinate transform applied after motion offset
 * @param transf Final transform including offset applied in space frame
 * @return Offset motion vector in space frame
 */
MotionVector
coord_transform_get_space_offset(const Transform& transf_post, const Transform& transf);

/**
 * @brief Apply motion vector offset to transform in mobile frame
 *
 * @param transf_initial Initial transform
 * @param offset Offset motion vector of final transform with respect to initial transform
 * @return Final transform with mobile frame offset applied
 */
Transform
coord_transform_apply_body_offset(const Transform& transf_initial, const MotionVector& offset);

/**
 * @brief Get motion vector offset between two coordinate transforms in mobile frame
 *
 * @param transf_initial Initial transform
 * @param transf_final Final transform with mobile frame offset applied
 * @return Offset motion vector of final transform with respect to initial transform
 */
MotionVector
coord_transform_get_body_offset(const Transform& transf_initial, const Transform& transf_final);

/**
 * @brief Get velocity of child frame with respect to a common frame given relative velocity between
 * parent and child frames
 *
 * @param parent_transf Transform of parent frame with respect to common frame
 * @param parent_velocity Velocity of parent frame with respect to common frame
 * @param motion_transf Transform of child frame with respect to parent frame
 * @param motion_velocity Velocity of child frame with respect to parent frame
 * @return Velocity of child frame with respect to common frame
 */
MotionVector motion_transform_velocity(
    const Transform& parent_transf, const MotionVector& parent_velocity,
    const Transform& motion_transf, const MotionVector& motion_velocity);

/**
 * @brief Get acceleration of child frame with respect to a common frame given relative motion
 * between parent and child frames
 *
 * @param parent_transf Transform of parent frame with respect to common frame
 * @param parent_velocity Velocity of parent frame with respect to common frame
 * @param parent_acceleration Acceleration of parent frame with respect to common frame
 * @param motion_transf Transform of child frame with respect to parent frame
 * @param motion_velocity Velocity of child frame with respect to parent frame
 * @param motion_acceleration Acceleration of child frame with respect to parent frame
 * @return Acceleration of child frame with respect to common frame
 */
MotionVector motion_transform_acceleration(
    const Transform& parent_transf, const MotionVector& parent_velocity,
    const MotionVector& parent_acceleration, const Transform& motion_transf,
    const MotionVector& motion_velocity, const MotionVector& motion_acceleration);

/**
 * @brief Space to body-fixed velocity
 *
 * @param pose Pose of coordinate frame
 * @param space_velocity Velocity in space frame
 * @return Velocity in body-fixed frame
 */
MotionVector
motion_transform_space_to_body_velocity(const Transform& pose, const MotionVector& space_velocity);

/**
 * @brief Body-fixed to space frame velocity
 *
 * @param pose Pose of coordinate frame
 * @param body_velocity Velocity in body-fixed frame
 * @return Velocity in space frame
 */
MotionVector
motion_transform_body_to_space_velocity(const Transform& pose, const MotionVector& body_velocity);

/**
 * @brief Get velocity of child frame with respect to a common frame given transform between parent
 * and child frames
 *
 * @param parent_transf Transform of parent frame with respect to common frame
 * @param parent_velocity Velocity of parent frame with respect to common frame
 * @param child_position Position of child frame with respect to parent frame
 * @return Velocity of child frame with respect to common frame
 */
MotionVector motion_transform_velocity_position(
    const Transform& parent_transf, const MotionVector& parent_velocity,
    const Vec3& child_position);

/**
 * @brief Get acceleration of child frame with respect to a common frame given transform between
 * parent and child frames
 *
 * @param parent_transf Transform of parent frame with respect to common frame
 * @param parent_velocity Velocity of parent frame with respect to common frame
 * @param parent_acceleration Acceleration of parent frame with respect to common frame
 * @param child_position Position of child frame with respect to parent frame
 * @return Acceleration of child frame with respect to common frame
 */
MotionVector motion_transform_acceleration_position(
    const Transform& parent_transf, const MotionVector& parent_velocity,
    const MotionVector& parent_acceleration, const Vec3& child_position);

/**
 * @}
 */

} // namespace fsb

#endif
