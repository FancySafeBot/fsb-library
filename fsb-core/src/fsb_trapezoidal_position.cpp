
#include <cmath>
#include "fsb_trajectory_types.h"
#include "fsb_types.h"
#include "fsb_trapezoidal_position.h"

namespace fsb
{

TrapezoidalStatus TrapezoidalPosition::generate(
    const Real start_time, const Real initial_position, const Real final_position,
    const Real max_velocity, const Real max_acceleration, const Real max_jerk)
{
    auto status = TrapezoidalStatus::SUCCESS;

    if ((max_velocity < FSB_TOL) || (max_acceleration < FSB_TOL) || (max_jerk < FSB_TOL))
    {
        return TrapezoidalStatus::MAX_VALUE_BELOW_TOLERANCE;
    }

    m_start_time = start_time;
    m_initial_state = {initial_position, 0.0, 0.0, 0.0};
    m_final_state = {final_position, 0.0, 0.0, 0.0};

    const Real position_change = final_position - initial_position;
    const Real abs_position_change = fabs(position_change);
    const Real sign = (position_change >= 0.0) ? 1.0 : -1.0;

    if (abs_position_change < FSB_TOL)
    {
        // Zero displacement: no motion
        m_total_duration = 0.0;
        const TrajState zero_state = {initial_position, 0.0, 0.0, 0.0};
        m_accel_ramp.goto_velocity(start_time, zero_state, 0.0, 0.0, max_acceleration, max_jerk);
        m_cruise.generate(start_time, 0.0, zero_state, 0.0);
        m_decel_ramp.goto_velocity(start_time, zero_state, 0.0, 0.0, max_acceleration, max_jerk);
        m_seg_extrapolate.generate(start_time, 0.0, zero_state, 0.0);
        return status;
    }

    // Displacement covered by a single jerk-limited velocity ramp from v=0 to v=vPeak
    // (starting and ending with zero acceleration) using the Ruckig/Berscheid-Kroger algorithm.
    //
    // Two cases depending on whether the max acceleration plateau is reached:
    //
    // Threshold velocity: v_thresh = aMax^2 / jMax
    //
    // Case A - plateau (vPeak >= v_thresh):
    //   t_ramp = vPeak/aMax + aMax/jMax
    //   Since the velocity profile rises symmetrically from 0 to vPeak,
    //   the average velocity is vPeak/2, so:
    //   d_accel = (vPeak/2) * t_ramp = vPeak^2/(2*aMax) + vPeak*aMax/(2*jMax)
    //
    // Case B - triangle (vPeak < v_thresh):
    //   t_ramp = 2 * sqrt(vPeak/jMax)
    //   d_accel = (vPeak/2) * t_ramp = vPeak * sqrt(vPeak/jMax)
    //
    // The total accel+decel distance (symmetric problem, same ramp both ways):
    //   d_total = 2 * d_accel
    //
    // Solving for vPeak given d_total = D (no-cruise case):
    //   Case B: vPeak = (D * sqrt(jMax) / 2)^(2/3)
    //   Case A: vPeak^2/aMax + vPeak*aMax/jMax = D
    //           => vPeak = (-aMax^2/jMax + sqrt((aMax^2/jMax)^2 + 4*aMax*D)) / 2

    const Real v_thresh = (max_acceleration * max_acceleration) / max_jerk;

    // Compute d_accel at max_velocity to check if it can be reached
    const Real d_accel_at_vmax = (max_velocity >= v_thresh)
        ? (max_velocity * max_velocity / (2.0 * max_acceleration)
           + max_velocity * max_acceleration / (2.0 * max_jerk))
        : (max_velocity * sqrt(max_velocity / max_jerk));
    const Real d_no_cruise = 2.0 * d_accel_at_vmax;

    Real v_peak = 0.0;
    Real t_cruise = 0.0;

    if (abs_position_change >= d_no_cruise)
    {
        // Max velocity is reachable; add a cruise phase
        v_peak = max_velocity;
        t_cruise = (abs_position_change - d_no_cruise) / max_velocity;
    }
    else
    {
        // Max velocity not reachable; find peak velocity analytically
        // Try triangle regime first
        const Real v_tri = pow((abs_position_change / 2.0) * sqrt(max_jerk), 2.0 / 3.0);
        if (v_tri < v_thresh)
        {
            // Triangle profile: no acceleration plateau
            v_peak = v_tri;
        }
        else
        {
            // Plateau profile: solve vPeak^2/(aMax) + vPeak*(aMax/jMax) = D
            // Rearranged: vPeak^2 + vPeak*(aMax^2/jMax) - D*aMax = 0
            // vPeak = (-(aMax^2/jMax) + sqrt((aMax^2/jMax)^2 + 4*D*aMax)) / 2
            const Real b = max_acceleration * max_acceleration / max_jerk;
            const Real discriminant = b * b + 4.0 * abs_position_change * max_acceleration;
            v_peak = (-b + sqrt(discriminant)) / 2.0;
        }
        t_cruise = 0.0;
    }

    // Build the actual trajectory
    // Accel ramp: (pos=initial_position, v=0, a=0) -> (v=sign*v_peak, a=0)
    const TrajState initial_state_actual = {initial_position, 0.0, 0.0, 0.0};
    status = m_accel_ramp.goto_velocity(
        start_time, initial_state_actual, sign * v_peak, 0.0, max_acceleration, max_jerk);

    if (status != TrapezoidalStatus::SUCCESS)
    {
        return status;
    }

    // Cruise phase at constant velocity
    const TrajState after_accel = m_accel_ramp.get_final_state();
    const Real accel_end_time = m_accel_ramp.get_final_time();
    m_cruise.generate(accel_end_time, t_cruise, after_accel, sign * v_peak);

    // Decel ramp: (v=sign*v_peak, a=0) -> (v=0, a=0)
    const TrajState after_cruise = m_cruise.get_final_state();
    const Real cruise_end_time = m_cruise.get_final_time();
    status = m_decel_ramp.goto_velocity(
        cruise_end_time, after_cruise, 0.0, 0.0, max_acceleration, max_jerk);

    if (status != TrapezoidalStatus::SUCCESS)
    {
        return status;
    }

    const TrajState traj_final_state = m_decel_ramp.get_final_state();
    const Real decel_end_time = m_decel_ramp.get_final_time();
    m_seg_extrapolate.generate(decel_end_time, 0.0, traj_final_state, 0.0);

    m_total_duration = decel_end_time - start_time;

    return status;
}
//
// TrapezoidalStatus TrapezoidalPosition::generate(
//     Real start_time, Real initial_position, Real final_position,
//     Real initial_velocity, Real final_velocity,
//     Real max_velocity, Real max_acceleration, Real max_jerk)
// {
//
// }

TrajState TrapezoidalPosition::evaluate(Real t_eval) const
{
    TrajState result = {};
    if (t_eval < m_start_time)
    {
        result = m_initial_state;
    }
    else if (t_eval <= m_accel_ramp.get_final_time())
    {
        result = m_accel_ramp.evaluate(t_eval);
    }
    else if (t_eval <= m_cruise.get_final_time())
    {
        result = m_cruise.evaluate(t_eval);
    }
    else if (t_eval <= m_start_time + m_total_duration)
    {
        result = m_decel_ramp.evaluate(t_eval);
    }
    else
    {
        result = m_seg_extrapolate.evaluate(t_eval);
    }
    return result;
}

} // namespace fsb
