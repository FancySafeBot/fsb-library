#pragma once

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
enum class TrapezoidalStatus
{
    /**
     * @brief Generation of trapezoidal trajectory is successful
     */
    SUCCESS,
    /**
     * @brief Maximum values not positive
     */
    MAX_VALUE_BELOW_TOLERANCE,
    /**
     * @brief Unable to reach target with current state
     */
    FAILED_TRAJECTORY_GENERATION
};

/**
 * @brief Duration of different phases in a trapezoidal profile
 */
struct TrapezoidalDuration
{
    /** Duration of the start phase */
    real_t start;

    /** Duration of the plateau phase */
    real_t plateau;

    /** Duration of the end phase */
    real_t end;
};

/**
 * @brief Constraints for a trapezoidal profile
 */
struct TrapezoidalConstraints
{
    /** Jerk at the start of the profile */
    real_t start_jerk;

    /** Acceleration during the plateau phase */
    real_t plateau_acceleration;

    /** Jerk at the end of the profile */
    real_t end_jerk;
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
        real_t start_time, const TrajState& initial_state, real_t final_velocity,
        real_t final_acceleration, real_t max_acceleration, real_t max_jerk);

    /**
     * @brief Evaluate trajectory
     *
     * @param t_eval Evaluation time
     * @return Trajectory state at evaluation time
     */
    [[nodiscard]] TrajState evaluate(real_t t_eval) const override;

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
    [[nodiscard]] real_t get_start_time() const override
    {
        return m_start_time;
    }

    /**
     * @brief Get total duration.
     *
     * @return Total duration of trajectory
     */
    [[nodiscard]] real_t get_duration() const override
    {
        return m_total_duration;
    }

    /**
     * @brief Get final time of trajectory.
     *
     * @return Final time
     */
    [[nodiscard]] real_t get_final_time() const override
    {
        return m_start_time + m_total_duration;
    }

private:
    real_t m_start_time = 0.0;
    real_t m_total_duration = 0.0;

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
