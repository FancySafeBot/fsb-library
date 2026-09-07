#ifndef FSB_TRAPEZOIDAL_POSITION_H
#define FSB_TRAPEZOIDAL_POSITION_H

#include "fsb_types.h"
#include "fsb_trajectory_types.h"
#include "fsb_trajectory_segment.h"
#include "fsb_trapezoidal_velocity.h"

namespace fsb
{

/**
 * @defgroup TopicTrapezoidal Trapezoidal trajectory profile
 * @{
 */

/**
 * Trapezoidal velocity motion profile
 */
class TrapezoidalPosition final : public SegmentScalar
{
public:
    TrapezoidalPosition() = default;

    /**
     * @brief Goto target position
     *
     * @param start_time
     * @param initial_position
     * @param final_position
     * @param max_acceleration
     * @param max_jerk
     * @return
     */
    TrapezoidalStatus generate(
        Real start_time, Real initial_position, Real final_position,
        Real max_velocity, Real max_acceleration, Real max_jerk);

    /**
     * @brief Goto target position with initial and final velocities
     *
     * @param start_time
     * @param initial_position
     * @param final_position
     * @param initial_velocity
     * @param final_velocity
     * @param max_velocity
     * @param max_acceleration
     * @param max_jerk
     * @return
     */
    TrapezoidalStatus generate(
        Real start_time, Real initial_position, Real final_position,
        Real initial_velocity, Real final_velocity,
        Real max_velocity, Real max_acceleration, Real max_jerk);

    /**
     * @brief Evaluate trajectory
     *
     * @param t_eval Evaluation time
     * @return Trajectory state at evaluation time
     */
    [[nodiscard]] TrajState evaluate(Real t_eval) const override final;

    /**
     * @brief Get final state of segment.
     * @return Get final state at end of segment
     */
    [[nodiscard]] TrajState get_final_state() const override final
    {
        return evaluate(m_start_time + m_total_duration);
    }

    /**
     * @brief Get initial state of segment.
     * @return Get initial state at start of segment
     */
    [[nodiscard]] TrajState get_initial_state() const override final
    {
        return evaluate(m_start_time);
    }

    /**
     * @brief Get start time of segment.
     * @return Start time.
     */
    [[nodiscard]] Real get_start_time() const override final
    {
        return m_start_time;
    }

    /**
     * @brief Get total duration.
     *
     * @return Total duration of trajectory
     */
    [[nodiscard]] Real get_duration() const override final
    {
        return m_total_duration;
    }

    /**
     * @brief Get final time of trajectory.
     *
     * @return Final time
     */
    [[nodiscard]] Real get_final_time() const override final
    {
        return m_start_time + m_total_duration;
    }

private:
    Real m_start_time = 0.0;
    Real m_total_duration = 0.0;

    TrapezoidalVelocity m_accel_ramp;
    SegmentConstVel     m_cruise;
    TrapezoidalVelocity m_decel_ramp;
    SegmentConstVel     m_seg_extrapolate;

    TrajState m_initial_state = {};
    TrajState m_final_state = {};
};

/**
 * @}
 */

} // namespace fsb

#endif
