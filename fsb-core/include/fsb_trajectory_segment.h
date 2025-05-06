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
 * @brief Abstract class for segments
 */
class SegmentBase
{
public:
    virtual ~SegmentBase() = default;

    /**
     * @brief Get start time of the segment.
     * @return Start time.
     */
    [[nodiscard]] virtual real_t get_start_time() const = 0;

    /**
     * @brief Get duration of the segment.
     * @return Duration.
     */
    [[nodiscard]] virtual real_t get_duration() const = 0;

    /**
     * @brief Get final time of the segment.
     * @return Final time.
     */
    [[nodiscard]] virtual real_t get_final_time() const = 0;
};

/**
 * @brief Abstract class for scalar motion
 */
class Segment : public SegmentBase
{
public:
    /**
     * @brief Evaluate the segment at a given time.
     * @param[in] t_eval Time at which to evaluate the segment.
     * @return Trajectory state at the given time.
     */
    [[nodiscard]] virtual TrajState evaluate(real_t t_eval) const = 0;

    /**
     * @brief Get final state of the segment.
     * @return Final state at the end of the segment.
     */
    [[nodiscard]] virtual TrajState get_final_state() const = 0;
};

/**
 * @brief Abstract class for motion in x, y, and z coordinates
 */
class Segment3 : public SegmentBase
{
public:
    /**
     * @brief Evaluate the segment at a given time.
     * @param[in] t_eval Time at which to evaluate the segment.
     * @return Trajectory state in 3D at the given time.
     */
    [[nodiscard]] virtual TrajState3 evaluate(real_t t_eval) const = 0;

    /**
     * @brief Get final state of the segment.
     * @return Final state in 3D at the end of the segment.
     */
    [[nodiscard]] virtual TrajState3 get_final_state() const = 0;
};

/**
 * @brief Abstract class for motion in Cartesian coordinates
 */
class Segment6 : public SegmentBase
{
public:
    /**
     * @brief Evaluate the segment at a given time.
     * @param[in] t_eval Time at which to evaluate the segment.
     * @return Cartesian position, velocity, and acceleration at the given time.
     */
    [[nodiscard]] virtual CartesianPva evaluate(real_t t_eval) const = 0;

    /**
     * @brief Get final state of the segment.
     * @return Final Cartesian position, velocity, and acceleration at the end of the segment.
     */
    [[nodiscard]] virtual CartesianPva get_final_state() const = 0;
};

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
    void generate(real_t start_time, real_t duration, const TrajState& initial_state, real_t jerk);

    /**
     * @brief Evaluate segment trajectory
     *
     * @param[in] t_eval Evaluation time
     * @return Current trajectory state
     */
    [[nodiscard]] TrajState evaluate(real_t t_eval) const override;

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
    [[nodiscard]] real_t get_start_time() const override
    {
        return m_start_time;
    }

    /**
     * @brief Get duration.
     *
     * @return Duration of segment
     */
    [[nodiscard]] real_t get_duration() const override
    {
        return m_duration;
    }

    /**
     * @brief Get final time of segment.
     *
     * @return Final time
     */
    [[nodiscard]] real_t get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    real_t m_initial_position = 0.0; ///< Initial position
    real_t m_initial_velocity = 0.0; ///< Initial velocity
    real_t m_initial_acceleration = 0.0; ///< Initial acceleration
    real_t m_jerk = 0.0; ///< Constant jerk
    real_t m_start_time = 0.0; ///< Start time
    real_t m_duration = 0.0; ///< Duration
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
    void generate(real_t start_time, real_t duration, const TrajState& initial_state, real_t acceleration);

    /**
     * @brief Evaluate constant acceleration segment.
     *
     * @param[in] t_eval Evaluation time
     * @return Trajectory state at time of evaluation
     */
    [[nodiscard]] TrajState evaluate(real_t t_eval) const override;

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
    [[nodiscard]] real_t get_start_time() const override
    {
        return m_start_time;
    }

    /**
     * @brief Get duration.
     *
     * @return Duration of segment
     */
    [[nodiscard]] real_t get_duration() const override
    {
        return m_duration;
    }

    /**
     * @brief Get final time of segment.
     *
     * @return Final time
     */
    [[nodiscard]] real_t get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    real_t m_initial_position = 0.0; ///< Initial position
    real_t m_initial_velocity = 0.0; ///< Initial velocity
    real_t m_acceleration = 0.0; ///< Constant acceleration
    real_t m_start_time = 0.0; ///< Start time
    real_t m_duration = 0.0; ///< Duration
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
    void generate(real_t start_time, real_t duration, const TrajState& initial_state, real_t velocity);

    /**
     * @brief Evaluate constant velocity segment.
     *
     * @param[in] t_eval Evaluation time
     * @return Trajectory state at time of evaluation
     */
    [[nodiscard]] TrajState evaluate(real_t t_eval) const override;

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
    [[nodiscard]] real_t get_start_time() const override
    {
        return m_start_time;
    }

    /**
     * @brief Get duration.
     *
     * @return Duration of segment
     */
    [[nodiscard]] real_t get_duration() const override
    {
        return m_duration;
    }

    /**
     * @brief Get final time of segment.
     *
     * @return Final time
     */
    [[nodiscard]] real_t get_final_time() const override
    {
        return m_start_time + m_duration;
    }

private:
    real_t m_initial_position = 0.0; ///< Initial position
    real_t m_velocity = 0.0; ///< Constant velocity
    real_t m_start_time = 0.0; ///< Start time
    real_t m_duration = 0.0; ///< Duration
};

/**
 * @}
 */

} // namespace fsb

#endif
