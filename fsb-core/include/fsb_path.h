#ifndef FSB_TRAJECTORY_PATH_H
#define FSB_TRAJECTORY_PATH_H

#include "fsb_trajectory_types.h"
#include "fsb_types.h"

#include "fsb_motion.h"

namespace fsb
{

/**
 * @defgroup TopicPath Cartesian Path
 * @brief Path trajectories in Cartesian space
 * @{
 */

struct PathMotion
{
    Real displacement = 0.0;
    Vec3 direction = {};
    Real velocity = 0.0;
    Real acceleration_normal = 0.0;
    Real acceleration_tangential = 0.0;
};

struct PathPva
{
    CartesianPva pva = {};
    PathMotion motion = {};
};

class Path final : public Segment<PathPva>
{
public:

    [[nodiscard]] Real get_start_time() const override final
    {
        return m_start_time;
    }

    [[nodiscard]] Real get_duration() const override final
    {
        return m_duration;
    }

    [[nodiscard]] Real get_final_time() const override final
    {
        return m_start_time + m_duration;
    }

    /**
     * @brief Evaluate the segment at a given time.
     * @param[in] t_eval Time at which to evaluate the segment.
     * @return Cartesian position, velocity, and acceleration at the given time.
     */
    PathPva evaluate(Real t_eval) const override final;

    /**
     * @brief Get final state of the segment.
     * @return Final Cartesian position, velocity, and acceleration at the end of the segment.
     */
    PathPva get_final_state() const override final;

    [[nodiscard]] PathPva get_initial_state() const override final
    {
        return evaluate(m_start_time);
    }

private:
    CartesianPva m_initial = {};
    CartesianPva m_final = {};

    Real m_start_displacement = 0.0;
    Real m_arc_length = 0.0;
    Real m_start_time = 0.0;
    Real m_duration = 0.0;
};

/**
 * @}
 */

} // namespace fsb

#endif // FANCYSAFEBOT_FSB_PATH_H
