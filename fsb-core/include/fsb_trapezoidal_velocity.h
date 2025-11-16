#ifndef FSB_TRAPEZOIDAL_VELOCITY_H
#define FSB_TRAPEZOIDAL_VELOCITY_H

#include <cstdint>
#include "fsb_types.h"
#include "fsb_trajectory_types.h"
#include "fsb_trajectory_segment.h"

namespace fsb
{

/**
 * @defgroup TopicTrapezoidal Trapezoidal trajectory profile
 * @{
 */

/**
 * @brief Result of attempting to compute trapezoidal trajectory
 */
enum class TrapezoidalStatus: uint8_t
{
    /**
     * @brief Generation of trapezoidal trajectory is successful
     */
    SUCCESS = 0,
    /**
     * @brief Maximum values not positive
     */
    MAX_VALUE_BELOW_TOLERANCE = 1,
    /**
     * @brief Unable to reach target with current state
     */
    FAILED_TRAJECTORY_GENERATION = 2
};

/**
 * @brief Duration of different phases in a trapezoidal profile
 */
struct TrapezoidalDuration
{
    /** Duration of the start phase */
    Real start;

    /** Duration of the plateau phase */
    Real plateau;

    /** Duration of the end phase */
    Real end;
};

/**
 * @brief Constraints for a trapezoidal profile
 */
struct TrapezoidalConstraints
{
    /** Jerk at the start of the profile */
    Real start_jerk;

    /** Acceleration during the plateau phase */
    Real plateau_acceleration;

    /** Jerk at the end of the profile */
    Real end_jerk;
};

/**
 * Trapezoidal velocity motion profile
 */
class TrapezoidalVelocity final : public Segment
{
public:
    TrapezoidalVelocity() = default;

    /**
     * @brief Goto target velocity
     *
     * @param start_time
     * @param initial_state
     * @param final_velocity
     * @param final_acceleration
     * @param max_acceleration
     * @param max_jerk
     * @return
     */
    TrapezoidalStatus goto_velocity(
        Real start_time, const TrajState& initial_state, Real final_velocity,
        Real final_acceleration, Real max_acceleration, Real max_jerk);

    /**
     * @brief Evaluate trajectory
     *
     * @param t_eval Evaluation time
     * @return Trajectory state at evaluation time
     */
    [[nodiscard]] TrajState evaluate(Real t_eval) const override;

    /**
     * @brief Get final state of segment.
     * @return Get final state at end of segment
     */
    [[nodiscard]] TrajState get_final_state() const override
    {
        return evaluate(m_start_time + m_total_duration);
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
        return m_total_duration;
    }

    /**
     * @brief Get final time of trajectory.
     *
     * @return Final time
     */
    [[nodiscard]] Real get_final_time() const override
    {
        return m_start_time + m_total_duration;
    }

private:
    Real m_start_time = 0.0;
    Real m_total_duration = 0.0;

    SegmentConstJerk m_seg1;
    SegmentConstAcc  m_seg2;
    SegmentConstJerk m_seg3;
    SegmentConstAcc  m_seg_extrapolate;

    TrajState m_initial_state = {};
    TrajState m_final_state = {};
};

/**
 * @}
 */

} // namespace fsb

#endif
