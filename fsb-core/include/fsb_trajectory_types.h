#ifndef FSB_TRAJECTORY_TYPES_H
#define FSB_TRAJECTORY_TYPES_H

#include "fsb_types.h"
#include "fsb_motion.h"

namespace fsb
{

/**
 * @defgroup TopicTrajectorySegment Small segment trajectories
 * @brief Segments with motion constraints
 * @{
 */

/**
 * @brief Trajectory scalar motion state
 */
struct TrajState
{
    /**
     * @brief Position
     */
    Real position;
    /**
     * @brief Velocity
     */
    Real velocity;
    /**
     * @brief Acceleration
     */
    Real acceleration;
    /**
     * @brief Jerk
     */
    Real jerk;
};

/**
 * @brief Trajectory vector motion state
 */
struct TrajState3
{
    /**
     * @brief Position
     */
    Vec3 position;
    /**
     * @brief Velocity
     */
    Vec3 velocity;
    /**
     * @brief Acceleration
     */
    Vec3 acceleration;
    /**
     * @brief Jerk
     */
    Vec3 jerk;
};

/**
 * @brief Abstract class for segments
 */
template <typename TrajStateType>
class Segment
{
public:
    virtual ~Segment() = default;

    /**
     * @brief Get start time of the segment.
     * @return Start time.
     */
    [[nodiscard]] virtual Real get_start_time() const = 0;

    /**
     * @brief Get duration of the segment.
     * @return Duration.
     */
    [[nodiscard]] virtual Real get_duration() const = 0;

    /**
     * @brief Get final time of the segment.
     * @return Final time.
     */
    [[nodiscard]] virtual Real get_final_time() const = 0;

    /**
     * @brief Evaluate the segment at a given time.
     * @param[in] t_eval Time at which to evaluate the segment.
     * @return Trajectory state at the given time.
     */
    [[nodiscard]] virtual TrajStateType evaluate(Real t_eval) const = 0;

    /**
     * @brief Get final state of the segment.
     * @return Final state at the end of the segment.
     */
    [[nodiscard]] virtual TrajStateType get_final_state() const = 0;

    /**
     * @brief Get initial state of the segment.
     * @return Initial state at the beginning of the segment.
     */
    [[nodiscard]] virtual TrajStateType get_initial_state() const = 0;
};

using SegmentScalar = Segment<TrajState>;
using Segment3 = Segment<TrajState3>;
using SegmentCartesian = Segment<CartesianPva>;

/**
 * @}
 */

} // namespace fsb

#endif
