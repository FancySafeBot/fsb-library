#ifndef FSB_TRAJECTORY_SEGMENT_H
#define FSB_TRAJECTORY_SEGMENT_H

#include "fsb_motion.h"
#include "fsb_trajectory_types.h"
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup TopicTrajectorySegment Small segment trajectories
 * @brief Segments with motion constraints
 * @{
 */


/**
 * @brief Constant jerk profile
 */
class SegmentConstJerk final : public Segment
{
public:
    SegmentConstJerk() = default;

    /**
     * @brief Generate constant jerk trajectory
     *
     * @param[in] start_time Start time
     * @param[in] duration Segment duration
     * @param[in] initial_state Initial state of trajectory
     * @param[in] jerk Constant jerk constraint
     */
    void generate(Real start_time, Real duration, const TrajState& initial_state, Real jerk);

    /**
     * @brief Evaluate segment trajectory
     *
     * @param[in] t_eval Evaluation time
     * @return Current trajectory state
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
     * @brief Get duration.
     *
     * @return Duration of segment
     */
    [[nodiscard]] Real get_duration() const override
    {
        return m_duration;
    }

    /**
     * @brief Get final time of segment.
     *
     * @return Final time
     */
    [[nodiscard]] Real get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    Real m_initial_position = 0.0; ///< Initial position
    Real m_initial_velocity = 0.0; ///< Initial velocity
    Real m_initial_acceleration = 0.0; ///< Initial acceleration
    Real m_jerk = 0.0; ///< Constant jerk
    Real m_start_time = 0.0; ///< Start time
    Real m_duration = 0.0; ///< Duration
};

/**
 * @brief Constant acceleration profile
 */
class SegmentConstAcc final : public Segment
{
public:
    SegmentConstAcc() = default;

    /**
     * @brief Generate constant acceleration profile
     *
     * @param[in] start_time Start time of segment trajectory
     * @param[in] duration Duration of segment
     * @param[in] initial_state Initial state of segment
     * @param[in] acceleration Constant acceleration constraint
     */
    void generate(Real start_time, Real duration, const TrajState& initial_state, Real acceleration);

    /**
     * @brief Evaluate constant acceleration segment.
     *
     * @param[in] t_eval Evaluation time
     * @return Trajectory state at time of evaluation
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
     * @brief Get duration.
     *
     * @return Duration of segment
     */
    [[nodiscard]] Real get_duration() const override
    {
        return m_duration;
    }

    /**
     * @brief Get final time of segment.
     *
     * @return Final time
     */
    [[nodiscard]] Real get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    Real m_initial_position = 0.0; ///< Initial position
    Real m_initial_velocity = 0.0; ///< Initial velocity
    Real m_acceleration = 0.0; ///< Constant acceleration
    Real m_start_time = 0.0; ///< Start time
    Real m_duration = 0.0; ///< Duration
};

/**
 * @brief Constant velocity profile
 */
class SegmentConstVel final : public Segment
{
public:
    SegmentConstVel() = default;

    /**
     * @brief Generate constant velocity profile
     *
     * @param[in] start_time Start time of segment trajectory
     * @param[in] duration Duration of segment
     * @param[in] initial_state Initial state of segment
     * @param[in] velocity Constant velocity constraint
     */
    void generate(Real start_time, Real duration, const TrajState& initial_state, Real velocity);

    /**
     * @brief Evaluate constant velocity segment.
     *
     * @param[in] t_eval Evaluation time
     * @return Trajectory state at time of evaluation
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
     * @brief Get duration.
     *
     * @return Duration of segment
     */
    [[nodiscard]] Real get_duration() const override
    {
        return m_duration;
    }

    /**
     * @brief Get final time of segment.
     *
     * @return Final time
     */
    [[nodiscard]] Real get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    Real m_initial_position = 0.0; ///< Initial position
    Real m_velocity = 0.0; ///< Constant velocity
    Real m_start_time = 0.0; ///< Start time
    Real m_duration = 0.0; ///< Duration
};

/**
 * @}
 */

} // namespace fsb

#endif
