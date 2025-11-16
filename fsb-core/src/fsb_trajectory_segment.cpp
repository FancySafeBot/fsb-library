
#include "fsb_types.h"
#include "fsb_trajectory_segment.h"
#include "fsb_trajectory_types.h"

namespace fsb
{

void SegmentConstJerk::generate(
    const Real start_time, const Real duration, const TrajState& initial_state,
    const Real jerk)
{
    m_initial_position = initial_state.position;
    m_initial_velocity = initial_state.velocity;
    m_initial_acceleration = initial_state.acceleration;
    m_jerk = jerk;
    m_start_time = start_time;
    m_duration = duration;
}

TrajState SegmentConstJerk::evaluate(Real t_eval) const
{
    t_eval -= m_start_time;
    return {
        m_initial_position + m_initial_velocity * t_eval + 0.5 * m_initial_acceleration * t_eval * t_eval +
                      (1.0 / 6.0) * m_jerk * t_eval * t_eval * t_eval,
        m_initial_velocity + m_initial_acceleration * t_eval + 0.5 * m_jerk * t_eval * t_eval,
        m_initial_acceleration + m_jerk * t_eval,
        m_jerk
    };
}

void SegmentConstAcc::generate(
    const Real start_time, const Real duration, const TrajState& initial_state,
    const Real acceleration)
{
    m_initial_position = initial_state.position;
    m_initial_velocity = initial_state.velocity;
    m_acceleration = acceleration;
    m_start_time = start_time;
    m_duration = duration;
}

TrajState SegmentConstAcc::evaluate(Real t_eval) const
{
    t_eval -= m_start_time;
    return {
        m_initial_position + m_initial_velocity * t_eval + 0.5 * m_acceleration * t_eval * t_eval,
        m_initial_velocity + m_acceleration * t_eval,
        m_acceleration,
        0.0
    };
}

void SegmentConstVel::generate(
    const Real start_time, const Real duration, const TrajState& initial_state,
    const Real velocity)
{
    m_initial_position = initial_state.position;
    m_velocity = velocity;
    m_start_time = start_time;
    m_duration = duration;
}

TrajState SegmentConstVel::evaluate(Real t_eval) const
{
    t_eval -= m_start_time;
    return {
        m_initial_position + m_velocity * t_eval,
        m_velocity,
        0.0,
        0.0
    };
}

} // namespace fsb
