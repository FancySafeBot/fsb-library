
#ifndef FSB_SPATIAL_H
#define FSB_SPATIAL_H

#include "fsb_configuration.h"
#include "fsb_motion.h"
#include "fsb_jacobian.h"
#include <cstddef>

namespace fsb
{

/**
 * @defgroup TopicSpatial Spatial Algebra
 * @brief Spatial algebra.
 *
 * @{
 */

/**
 * @brief Convert coordinate transform to spatial transform
 *
 * @param transf Coordinate transform
 * @return Spatial transform
 */
Transform transform_to_spatial(const Transform& transf);

/**
 * @brief Convert inverse of a coordinate transform to spatial transform
 *
 * @param transf Coordinate transform
 * @return Inverse spatial transform
 */
Transform transform_inverse_to_spatial(const Transform& transf);

/**
 * @brief Convert spatial velocity from body frame to space frame.
 *
 * @param pose Pose of body
 * @param velocity_space Spatial velocity of body in space frame.
 * @return Velocity in body-fixed frame
 */
MotionVector spatial_space_to_body(const Transform& pose, const MotionVector& velocity_space);

/**
 * @brief Convert spatial velocity from space frame to body-fixed frame.
 *
 * @param pose Pose of body
 * @param velocity_body Spatial velocity of body in space frame.
 * @return Velocity in body-fixed frame
 */
MotionVector spatial_body_to_space(const Transform& pose, const MotionVector& velocity_body);

/**
 * @brief Convert from jacobian in body frame to space frame.
 *
 * @param pose Pose of body
 * @param jacobian Jacobian for pose in space frame.
 * @param columns Number of columns (degrees of freedom) of Jacobian matrix
 * @return Jacobian in body-fixed frame
 */
Jacobian spatial_jacobian_body_to_space(const Transform& pose, const Jacobian& jacobian, size_t columns = MaxSize::dofs);

/**
 * @brief Convert from jacobian in space frame to body-fixed frame.
 *
 * @param pose Pose of body
 * @param jacobian Jacobian for pose in space frame.
 * @param columns Number of columns (degrees of freedom) of Jacobian matrix
 * @return Jacobian in body-fixed frame
 */
Jacobian spatial_jacobian_space_to_body(const Transform& pose, const Jacobian& jacobian, size_t columns = MaxSize::dofs);

/**
 * @}
 */

} // namespace fsb

#endif // FSB_SPATIAL_H
