#include <cmath>
#include <cstddef>

#include "fsb_motion.h"
#include "fsb_configuration.h"
#include "fsb_types.h"
#include "fsb_kinematic_redundancy.h"
#include "fsb_linalg.h"
#include "fsb_jacobian.h"

namespace fsb
{

FsbLinalgErrorType
jacobian_pseudoinverse(const Jacobian& jacobian, Jacobian& inverse_jacobian, const size_t dofs)
{
    // lapack pseudoinverse c array inputs
    constexpr size_t work_len = MaxSize::linalg_work;
    double_t work[work_len] = {};
    // run pseudoinverse
    return fsb_linalg_pseudoinverse(jacobian.j.data(), FSB_CART_SIZE, dofs, work_len, work, inverse_jacobian.j.data());
}

JointSpace compute_nullspace_motion(
    const Jacobian& jacobian, const JointSpace& joint_motion, const size_t dofs)
{
    JointSpace result = {};
    // Calculate J+ * J * qd
    // First, compute J * qd (cartesian motion)
    MotionVector cart_motion = {};
    for (size_t col = 0; col < dofs; ++col)
    {
        cart_motion.angular.x += jacobian.j[jacobian_index(0U, col)] * joint_motion.qv[col];
        cart_motion.angular.y += jacobian.j[jacobian_index(1U, col)] * joint_motion.qv[col];
        cart_motion.angular.z += jacobian.j[jacobian_index(2U, col)] * joint_motion.qv[col];
        cart_motion.linear.x += jacobian.j[jacobian_index(3U, col)] * joint_motion.qv[col];
        cart_motion.linear.y += jacobian.j[jacobian_index(4U, col)] * joint_motion.qv[col];
        cart_motion.linear.z += jacobian.j[jacobian_index(5U, col)] * joint_motion.qv[col];
    }

    // Then, compute qn = J+ * (J * qd) = J+ * cart_motion
    // lapack pseudoinverse c array inputs
    double_t motion_vec[FSB_CART_SIZE] = {};
    double_t joint_null[MaxSize::dofs] = {};
    constexpr size_t nrhs = 1U;
    constexpr size_t work_len = 512U;
    double_t work[work_len] = {};
    // copy data to c array
    motion_vec[0] = cart_motion.angular.x;
    motion_vec[1] = cart_motion.angular.y;
    motion_vec[2] = cart_motion.angular.z;
    motion_vec[3] = cart_motion.linear.x;
    motion_vec[4] = cart_motion.linear.y;
    motion_vec[5] = cart_motion.linear.z;
    // run least squares solve
    FsbLinalgErrorType linalg_err = fsb_linalg_leastsquares_solve(
        jacobian.j.data(), FSB_CART_SIZE, dofs, motion_vec, nrhs,
        work_len, work, joint_null);

    // Compute nullspace motion: (I - J+ * J) * qd = qd - J+ * J * qd
    if (linalg_err == FsbLinalgErrorType::EFSB_LAPACK_ERROR_NONE)
    {
        for (size_t ind = 0U; ind < dofs; ++ind)
        {
            result.qv[ind] = joint_motion.qv[ind] - joint_null[ind];
        }
    }
    return result;
}

JointSpace compute_nullspace_motion(
    const Jacobian& jacobian, const Jacobian& inverse_jacobian, const JointSpace& joint_motion,
    const size_t dofs)
{
    JointSpace result = {};
    // Calculate J+ * J * qd
    // First, compute J * qd (cartesian motion)
    MotionVector cart_motion = {};
    for (size_t col = 0; col < dofs; ++col)
    {
        cart_motion.angular.x += jacobian.j[jacobian_index(0U, col)] * joint_motion.qv[col];
        cart_motion.angular.y += jacobian.j[jacobian_index(1U, col)] * joint_motion.qv[col];
        cart_motion.angular.z += jacobian.j[jacobian_index(2U, col)] * joint_motion.qv[col];
        cart_motion.linear.x += jacobian.j[jacobian_index(3U, col)] * joint_motion.qv[col];
        cart_motion.linear.y += jacobian.j[jacobian_index(4U, col)] * joint_motion.qv[col];
        cart_motion.linear.z += jacobian.j[jacobian_index(5U, col)] * joint_motion.qv[col];
    }

    // Then, compute J+ * (J * qd) = J+ * cart_motion
    JointSpace projection = {};
    for (size_t row = 0; row < dofs; ++row)
    {
        projection.qv[row]
            = inverse_jacobian.j[joint_matrix_index(row, 0U, dofs)] * cart_motion.angular.x
              + inverse_jacobian.j[joint_matrix_index(row, 1U, dofs)] * cart_motion.angular.y
              + inverse_jacobian.j[joint_matrix_index(row, 2U, dofs)] * cart_motion.angular.z
              + inverse_jacobian.j[joint_matrix_index(row, 3U, dofs)] * cart_motion.linear.x
              + inverse_jacobian.j[joint_matrix_index(row, 4U, dofs)] * cart_motion.linear.y
              + inverse_jacobian.j[joint_matrix_index(row, 5U, dofs)] * cart_motion.linear.z;
    }

    // Compute nullspace motion: (I - J+ * J) * qd = qd - J+ * J * qd
    for (size_t i = 0; i < dofs; ++i)
    {
        result.qv[i] = joint_motion.qv[i] - projection.qv[i];
    }

    return result;
}

} // namespace fsb
