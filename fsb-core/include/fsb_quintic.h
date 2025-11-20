
#ifndef FSB_QUINTIC_H
#define FSB_QUINTIC_H

#include "fsb_types.h"
#include "fsb_trajectory_types.h"

namespace fsb
{

/**
 * @defgroup TopicQuintic Quintic Polynomial Trajectory
 * @brief Trajectory using 5th order polynomials
 * @{
 */

/**
 * @brief Minimum duration for a quintic trajectory
 */
constexpr Real kQuinticMinDuration = 1e-6;

/**
 * @brief Coefficients of a quintic polynomial
 */
struct QuinticCoeffs
{
    Real c0 = 0.0; ///< Constant term
    Real c1 = 0.0; ///< Linear term
    Real c2 = 0.0; ///< Quadratic term
    Real c3 = 0.0; ///< Cubic term
    Real c4 = 0.0; ///< Quartic term
    Real c5 = 0.0; ///< Quintic term
};

/**
 * @brief Quintic trajectory
 */
class QuinticTrajectory final : public Segment
{
public:
    QuinticTrajectory() = default;

    /**
     * @brief Generate spline
     *
     * @param[in] start_time Start time of the trajectory
     * @param[in] duration Duration of the trajectory
     * @param[in] initial_state Initial state of the trajectory
     * @param[in] final_state Final state of the trajectory
     * @return true if the spline was successfully generated, false otherwise
     */
    bool generate(
        Real start_time, Real duration, const TrajState& initial_state,
        const TrajState& final_state);

    /**
     * @brief Evaluate position, velocity, acceleration, and jerk
     *
     * @param[in] t_eval Time at which to evaluate the trajectory
     * @return Trajectory state containing position, velocity, acceleration, and jerk at `t_eval`
     */
    [[nodiscard]] TrajState evaluate(Real t_eval) const override;

    /**
     * @brief Get final state of segment.
     * @return Get final state at end of segment
     */
    [[nodiscard]] TrajState get_final_state() const override
    {
        return evaluate(m_start_time + m_duration);
    }

    /**
     * @brief Get start time of segment.
     * @return Start time.
     */
    [[nodiscard]] Real get_start_time() const override
    {
        return m_start_time;
    }

    /**
     * @brief Get total duration.
     *
     * @return Total duration of trajectory
     */
    [[nodiscard]] Real get_duration() const override
    {
        return m_duration;
    }

    /**
     * @brief Get final time of trajectory.
     *
     * @return Final time
     */
    [[nodiscard]] Real get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    Real m_start_time = 0.0;
    Real m_duration = 0.0;

    QuinticCoeffs m_coeffs = {};
};

/**
 * @}
 */

} // namespace fsb

#endif
