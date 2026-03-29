#include <algorithm>
#include <cmath>
#include <cstddef>

#include "fsb_joint.h"
#include "fsb_motion.h"
#include "fsb_body_tree.h"
#include "fsb_configuration.h"
#include "fsb_types.h"
#include "fsb_kinematic_redundancy.h"
#include "fsb_linalg.h"
#include "fsb_jacobian.h"

namespace fsb
{

fsb::LinalgErrorType
jacobian_pseudoinverse(const Jacobian& jacobian, Jacobian& inverse_jacobian, const size_t dofs)
{
    // lapack pseudoinverse c array inputs
    constexpr size_t WorkLen = MaxSize::kLinalgWork;
    std::array<Real, WorkLen> work = {};
    // run pseudoinverse
    return fsb::linalg_pseudoinverse(
        fsb::Span<const Real>(jacobian.j.data(), FSB_CART_SIZE * dofs), FSB_CART_SIZE, dofs,
        fsb::Span<Real>(work.data(), WorkLen),
        fsb::Span<Real>(inverse_jacobian.j.data(), dofs * FSB_CART_SIZE));
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
        cart_motion.angular.x += jacobian.j[jacobian_index(0U, col)] * joint_motion[col];
        cart_motion.angular.y += jacobian.j[jacobian_index(1U, col)] * joint_motion[col];
        cart_motion.angular.z += jacobian.j[jacobian_index(2U, col)] * joint_motion[col];
        cart_motion.linear.x += jacobian.j[jacobian_index(3U, col)] * joint_motion[col];
        cart_motion.linear.y += jacobian.j[jacobian_index(4U, col)] * joint_motion[col];
        cart_motion.linear.z += jacobian.j[jacobian_index(5U, col)] * joint_motion[col];
    }

    // Then, compute qn = J+ * (J * qd) = J+ * cart_motion
    // lapack pseudoinverse c array inputs
    std::array<Real, FSB_CART_SIZE> motion_vec = {};
    std::array<Real, MaxSize::kDofs> joint_null = {};
    constexpr size_t NRHS = 1U;
    constexpr size_t WorkLen = 512U;
    std::array<Real, WorkLen> work = {};
    // copy data to c array
    motion_vec[0] = cart_motion.angular.x;
    motion_vec[1] = cart_motion.angular.y;
    motion_vec[2] = cart_motion.angular.z;
    motion_vec[3] = cart_motion.linear.x;
    motion_vec[4] = cart_motion.linear.y;
    motion_vec[5] = cart_motion.linear.z;
    // run least squares solve
    fsb::LinalgErrorType const linalg_err = fsb::linalg_leastsquares_solve(
        fsb::Span<const Real>(jacobian.j.data(), FSB_CART_SIZE * dofs), FSB_CART_SIZE, dofs,
        fsb::Span<const Real>(motion_vec.data(), FSB_CART_SIZE * NRHS), NRHS,
        fsb::Span<Real>(work.data(), WorkLen),
        fsb::Span<Real>(joint_null.data(), dofs * NRHS));

    // Compute nullspace motion: (I - J+ * J) * qd = qd - J+ * J * qd
    if (linalg_err == fsb::LinalgErrorType::ERROR_NONE)
    {
        for (size_t ind = 0U; ind < dofs; ++ind)
        {
            result[ind] = joint_motion[ind] - joint_null[ind];
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
        cart_motion.angular.x += jacobian.j[jacobian_index(0U, col)] * joint_motion[col];
        cart_motion.angular.y += jacobian.j[jacobian_index(1U, col)] * joint_motion[col];
        cart_motion.angular.z += jacobian.j[jacobian_index(2U, col)] * joint_motion[col];
        cart_motion.linear.x += jacobian.j[jacobian_index(3U, col)] * joint_motion[col];
        cart_motion.linear.y += jacobian.j[jacobian_index(4U, col)] * joint_motion[col];
        cart_motion.linear.z += jacobian.j[jacobian_index(5U, col)] * joint_motion[col];
    }

    // Then, compute J+ * (J * qd) = J+ * cart_motion
    JointSpace projection = {};
    for (size_t row = 0; row < dofs; ++row)
    {
        projection[row]
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
        result[i] = joint_motion[i] - projection[i];
    }

    return result;
}

JointSpace joint_limit_avoidance_objective(
    const JointSpacePosition& joint_positions, const JointLimits& joint_limits, size_t dofs,
    double gain)
{
    JointSpace result = {};

    const size_t max_dofs = MaxSize::kDofs;
    const size_t max_coords = MaxSize::kCoordinates;
    const size_t n_dofs = (dofs < max_dofs) ? dofs : max_dofs;
    const size_t n = (n_dofs < max_coords) ? n_dofs : max_coords;

    if ((n == 0U) || (!std::isfinite(gain)) || (gain == 0.0))
    {
        return result;
    }

    // Numerical safety: avoid division by 0 near limits.
    constexpr Real eps = 1e-9;

    for (size_t i = 0U; i < n; ++i)
    {
        if (!joint_limits.set[i])
        {
            continue;
        }

        const Real q = joint_positions[i];
        const Real qmin = joint_limits.lower_position[i];
        const Real qmax = joint_limits.upper_position[i];

        // Skip invalid/degenerate limits.
        if ((!std::isfinite(q)) || (!std::isfinite(qmin)) || (!std::isfinite(qmax))
            || (qmax <= qmin))
        {
            continue;
        }

        // Potential-field objective (matches the header documentation).
        Real const d_lower = std::max(q - qmin, eps);
        Real const d_upper = std::max(qmax - q, eps);

        const Real k = gain;
        Real qdot = k * ((1.0 / d_lower) - (1.0 / d_upper));

        // Respect configured max velocity if provided.
        if (const Real vmax = joint_limits.max_velocity[i]; std::isfinite(vmax) && (vmax > 0.0))
        {
            if (qdot > vmax)
            {
                qdot = vmax;
            }
            else if (qdot < -vmax)
            {
                qdot = -vmax;
            }
        }

        result[i] = qdot;
    }

    return result;
}

} // namespace fsb
