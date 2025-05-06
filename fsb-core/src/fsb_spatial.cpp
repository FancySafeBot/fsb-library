#include <cstddef>

#include "fsb_configuration.h"
#include "fsb_spatial.h"
#include "fsb_jacobian.h"
#include "fsb_motion.h"
#include "fsb_quaternion.h"
#include "fsb_rotation.h"
#include "fsb_types.h"

namespace fsb
{

Transform transform_to_spatial(const Transform& transf)
{
    const Vec3 pos = quat_rotate_vector(quat_conjugate(transf.rotation), transf.translation);
    return {
        transf.rotation, {-pos.x, -pos.y, -pos.z}
    };
}

Transform transform_inverse_to_spatial(const Transform& transf)
{
    return {quat_conjugate(transf.rotation), transf.translation};
}

MotionVector spatial_space_to_body(const Transform& pose, const MotionVector& velocity_space)
{
    const Mat3 rotmat = quat_to_rot(pose.rotation);
    const Vec3 cross_angular
        = rotate_mat3_transpose(rotmat, vector_cross(pose.translation, velocity_space.angular));
    return {
        rotate_mat3_transpose(rotmat, velocity_space.angular),
        vector_subtract(rotate_mat3_transpose(rotmat, velocity_space.linear), cross_angular)};
}

MotionVector spatial_body_to_space(const Transform& pose, const MotionVector& velocity_body)
{
    const Mat3 rotmat = quat_to_rot(pose.rotation);
    const Vec3 cross_angular
        = vector_cross(pose.translation, rotate_mat3(rotmat, velocity_body.angular));
    return {
        rotate_mat3(rotmat, velocity_body.angular),
        vector_add(cross_angular, rotate_mat3(rotmat, velocity_body.linear))};
}

Jacobian
spatial_jacobian_body_to_space(const Transform& pose, const Jacobian& jacobian, size_t columns)
{
    Jacobian result = {};
    if (columns > MaxSize::dofs)
    {
        columns = MaxSize::dofs;
    }

    for (size_t col = 0U; col < columns; ++col)
    {
        const MotionVector jac_col = {
            {jacobian.j[jacobian_index(0U, col)],
             jacobian.j[jacobian_index(1U, col)],
             jacobian.j[jacobian_index(2U, col)]},
            {jacobian.j[jacobian_index(3U, col)],
             jacobian.j[jacobian_index(4U, col)],
             jacobian.j[jacobian_index(5U,  col)]}
        };
        const MotionVector result_col = spatial_body_to_space(pose, jac_col);
        result.j[jacobian_index(0U, col)] = result_col.angular.x;
        result.j[jacobian_index(1U, col)] = result_col.angular.y;
        result.j[jacobian_index(2U, col)] = result_col.angular.z;
        result.j[jacobian_index(3U, col)] = result_col.linear.x;
        result.j[jacobian_index(4U, col)] = result_col.linear.y;
        result.j[jacobian_index(5U, col)] = result_col.linear.z;
    }

    return result;
}

Jacobian
spatial_jacobian_space_to_body(const Transform& pose, const Jacobian& jacobian, size_t columns)
{
    Jacobian result = {};
    if (columns > MaxSize::dofs)
    {
        columns = MaxSize::dofs;
    }

    for (size_t col = 0U; col < columns; ++col)
    {
        const MotionVector jac_col = {
            {jacobian.j[jacobian_index(0U, col)],
             jacobian.j[jacobian_index(1U, col)],
             jacobian.j[jacobian_index(2U, col)]},
            {jacobian.j[jacobian_index(3U, col)],
             jacobian.j[jacobian_index(4U, col)],
             jacobian.j[jacobian_index(5U, col)]}
        };
        const MotionVector result_col = spatial_space_to_body(pose, jac_col);
        result.j[jacobian_index(0U, col)] = result_col.angular.x;
        result.j[jacobian_index(1U, col)] = result_col.angular.y;
        result.j[jacobian_index(2U, col)] = result_col.angular.z;
        result.j[jacobian_index(3U, col)] = result_col.linear.x;
        result.j[jacobian_index(4U, col)] = result_col.linear.y;
        result.j[jacobian_index(5U, col)] = result_col.linear.z;
    }

    return result;
}

} // namespace fsb
