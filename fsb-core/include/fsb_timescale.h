#pragma once

#include "fsb_trajectory_types.h"
#include "fsb_types.h"
#include "fsb_trapezoidal_velocity.h"

namespace fsb
{

/**
 * @defgroup TopicTimescale Trajectory timescaling
 * @brief Scale trajectories with modified time differential
 * @{
 */

/**
 * @brief Apply timescale to trajectory position, velocity, and acceeleration
 *
 * @note The jerk is not scaled and is set to 0 on output
 *
 * @param[in] timescale Timescale state (position, velocity and acceleration)
 * @param[in] traj Trajectory state to be scaled
 * @return Scaled trajectory state.
 */
TrajState timescale_trajectory(const TrajState& timescale, const TrajState& traj);

/**
* @brief Result of timescale transition
*/
enum class TimescaleResult
{
    /**
     * @brief Timescale transition successful
     */
    SUCCESS,
    /**
     * @brief Timescale transition failed due to maximum timescale below tolerance
     */
    MAX_TIMESCALE_BELOW_TOLERANCE,
    /**
     * @brief Timescale transition failed
     */
    FAILED_TO_TRANSITION
};

/**
 * @brief Timescale trajectory with trapezoidal velocity profile
 */
class Timescale
{
public:
    Timescale() = default;

    /**
     * @brief Set timescale transition limits
     *
     * @param max_timescale Maximum timescale
     * @param max_timescale_deriv Maximum first derivative of timescale transition
     * @param max_timescale_2nd_deriv Maximum second derivative of timescale transition
     * @return
     */
    bool
    set_limits(real_t max_timescale, real_t max_timescale_deriv, real_t max_timescale_2nd_deriv);

    /**
     *
     * @param time_mono Monotonic clock time in seconds
     * @param time_scaled_init Initial scaled time in seconds
     * @param timescale Initial timescale value. Default 1.0
     */
    bool start(real_t time_mono, real_t time_scaled_init, real_t timescale = 1.0);

    /**
     * @brief Set new timescale
     *
     * @param time_mono Monotonic clock time in seconds
     * @param target_timescale Target timescale
     * @return
     */
    TimescaleResult goto_timescale(real_t time_mono, real_t target_timescale);

    /**
     *
     * @param time_mono Monotonic clock time in seconds
     * @return Scaled time state
     */
    [[nodiscard]] TrajState evaluate(real_t time_mono) const;

private:
    real_t m_max_timescale = 0.0;
    real_t m_max_timescale_deriv = 0.0;
    real_t m_max_timescale_2nd_deriv = 0.0;

    TrapezoidalVelocity m_time_traj;
};

/**
 * @}
 */

} // namespace fsb
