
#include <cmath>
#include "fsb_timescale.h"
#include "fsb_trapezoidal_velocity.h"
#include "fsb_types.h"
#include "fsb_trajectory_types.h"

namespace fsb
{

TrajState timescale_trajectory(const TrajState& timescale, const TrajState& traj)
{
    return {
        traj.position,
        traj.velocity * timescale.velocity,
        traj.acceleration * timescale.velocity * timescale.velocity
            + traj.velocity * timescale.acceleration,
        0.0};
}

bool Timescale::set_limits(
    const real_t max_timescale, const real_t max_timescale_deriv,
    const real_t max_timescale_2nd_deriv)
{
    bool success = true;
    if (max_timescale < 0.0 || max_timescale_deriv < FSB_TOL || max_timescale_2nd_deriv < FSB_TOL)
    {
        success = false;
    }
    else
    {
        m_max_timescale = max_timescale;
        m_max_timescale_deriv = max_timescale_deriv;
        m_max_timescale_2nd_deriv = max_timescale_2nd_deriv;
    }
    return success;
}

bool Timescale::start(const real_t time_mono, const real_t time_scaled_init, real_t timescale)
{
    if (fabs(timescale) > m_max_timescale)
    {
        timescale = (timescale < 0.0 ? -m_max_timescale : m_max_timescale);
    }
    const TrajState         initial_state = {time_scaled_init, timescale, 0.0, 0.0};
    constexpr real_t        target_timescale_deriv = 0.0;
    const TrapezoidalStatus status = m_time_traj.goto_velocity(
        time_mono,
        initial_state,
        timescale,
        target_timescale_deriv,
        m_max_timescale_deriv,
        m_max_timescale_2nd_deriv);
    return (status == TrapezoidalStatus::SUCCESS);
}

TimescaleResult Timescale::goto_timescale(const real_t time_mono, real_t target_timescale)
{
    const TrajState initial_state = m_time_traj.evaluate(time_mono);
    if (fabs(target_timescale) > m_max_timescale)
    {
        target_timescale = (target_timescale < 0.0 ? -m_max_timescale : m_max_timescale);
    }
    constexpr real_t target_timescale_deriv = 0.0;

    const TrapezoidalStatus status = m_time_traj.goto_velocity(
        time_mono,
        initial_state,
        target_timescale,
        target_timescale_deriv,
        m_max_timescale_deriv,
        m_max_timescale_2nd_deriv);
    return (status == TrapezoidalStatus::SUCCESS ? TimescaleResult::SUCCESS : TimescaleResult::FAILED_TO_TRANSITION);
}

TrajState Timescale::evaluate(const real_t time_mono) const
{
    return m_time_traj.evaluate(time_mono);
}

} // namespace fsb
