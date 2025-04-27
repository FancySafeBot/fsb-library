
#include <cmath>
#include "fsb_trajectory_types.h"
#include "fsb_types.h"
#include "fsb_trapezoidal_velocity.h"

namespace fsb
{

static TrapezoidalStatus generate_duration_fixed_plateau(
    real_t target_change, real_t initial_rate, real_t final_rate, real_t plateau_rate, real_t start_rate_deriv,
    real_t end_rate_deriv, TrapezoidalDuration& duration);

static TrapezoidalStatus generate_duration_no_plateau(
    real_t target_change, real_t initial_rate, real_t final_rate, real_t start_rate_deriv, real_t end_rate_deriv,
    real_t& peak_rate, TrapezoidalDuration& duration);

static TrapezoidalStatus generate_velocity_trapezoidal(
    real_t velocity_change, real_t initial_acceleration, real_t final_acceleration, real_t max_acceleration,
    real_t max_jerk, TrapezoidalConstraints& velocity_constraints, TrapezoidalDuration& duration);

static TrapezoidalStatus generate_velocity_positive_change(
    real_t velocity_change, real_t initial_acceleration, real_t final_acceleration,
    real_t max_acceleration, real_t max_jerk, TrapezoidalConstraints& velocity_constraints,
    TrapezoidalDuration& duration);

static TrapezoidalStatus generate_velocity_negative_change(
    real_t velocity_change, real_t initial_acceleration, real_t final_acceleration,
    real_t max_acceleration, real_t max_jerk, TrapezoidalConstraints& velocity_constraints,
    TrapezoidalDuration& duration);

static TrapezoidalStatus generate_duration_fixed_plateau(
        const real_t target_change,
        const real_t initial_rate, const real_t final_rate, const real_t plateau_rate,
        const real_t start_rate_deriv, const real_t end_rate_deriv, TrapezoidalDuration& duration)
{
    auto status = TrapezoidalStatus::SUCCESS;
    // ramp durations
    real_t duration_start = (plateau_rate - initial_rate) / start_rate_deriv;
    real_t duration_end = (final_rate - plateau_rate) / end_rate_deriv;
    if (fabs(duration_start) < FSB_TOL)
    {
        duration_start = 0.0;
    }
    if (fabs(duration_end) < FSB_TOL)
    {
        duration_end = 0.0;
    }
    // plateau duration numerator
    const real_t plateau_num =
        target_change - (initial_rate * duration_start + 0.5 * start_rate_deriv * duration_start * duration_start +
                         plateau_rate * duration_end + 0.5 * end_rate_deriv * duration_end * duration_end);
    // plateau duration
    real_t duration_plateau = plateau_num / plateau_rate;
    if (fabs(duration_plateau) < FSB_TOL)
    {
        duration_plateau = 0.0;
    }
    // all durations should be positive or 0
    if ((duration_start < 0.0) || (duration_plateau < 0.0) || (duration_end < 0.0))
    {
        status = TrapezoidalStatus::FAILED_TRAJECTORY_GENERATION;
    }
    else
    {
        duration.start = duration_start;
        duration.plateau = duration_plateau;
        duration.end = duration_end;
    }
    return status;
}

static TrapezoidalStatus generate_duration_no_plateau(
        const real_t target_change,
        const real_t initial_rate, const real_t final_rate,
        const real_t start_rate_deriv, const real_t end_rate_deriv,
        real_t& peak_rate, TrapezoidalDuration& duration)
{
    auto status = TrapezoidalStatus::SUCCESS;
    if (const real_t asqr = final_rate * final_rate + initial_rate * initial_rate
                            + 2.0 * start_rate_deriv * target_change;
        asqr < FSB_TOL)
    {
        status = TrapezoidalStatus::FAILED_TRAJECTORY_GENERATION;
    }
    else
    {
        // apply plateau rate
        const real_t plateau_rate_magnitude = 0.5 * sqrt(2.0 * asqr);
        peak_rate = (peak_rate < 0.0 ? -plateau_rate_magnitude : plateau_rate_magnitude);
        status = generate_duration_fixed_plateau(target_change, initial_rate, final_rate, peak_rate, start_rate_deriv, end_rate_deriv, duration);
    }
    return status;
}

static TrapezoidalStatus generate_velocity_trapezoidal(
    const real_t velocity_change, const real_t initial_acceleration, const real_t final_acceleration,
    const real_t max_acceleration, const real_t max_jerk,
    TrapezoidalConstraints& velocity_constraints,
    TrapezoidalDuration& duration)
{
    auto status = TrapezoidalStatus::SUCCESS;
    // Positive plateau acceleration
    const real_t start_jerk = max_jerk;
    const real_t end_jerk = -max_jerk;
    real_t plateau_acceleration = max_acceleration;
    // generate with max plateau rate
    status = generate_duration_fixed_plateau(velocity_change, initial_acceleration, final_acceleration,
                                   plateau_acceleration, start_jerk, end_jerk,
                                   duration);
    if (status != TrapezoidalStatus::SUCCESS)
    {
        // failed to reach max acceleration, try variable plateau acceleration
        status = generate_duration_no_plateau(
            velocity_change, initial_acceleration, final_acceleration, start_jerk, end_jerk,
            plateau_acceleration, duration);
    }
    if (status == TrapezoidalStatus::SUCCESS)
    {
        velocity_constraints.start_jerk = start_jerk;
        velocity_constraints.plateau_acceleration = plateau_acceleration;
        velocity_constraints.end_jerk = end_jerk;
    }
    return status;
}

static TrapezoidalStatus generate_velocity_positive_change(
    const real_t velocity_change, const real_t initial_acceleration, const real_t final_acceleration,
    const real_t max_acceleration, const real_t max_jerk, TrapezoidalConstraints& velocity_constraints,
    TrapezoidalDuration& duration)
{
    TrapezoidalStatus status = generate_velocity_trapezoidal(
        velocity_change, initial_acceleration, final_acceleration,
        max_acceleration, max_jerk, velocity_constraints, duration);
    if (status != TrapezoidalStatus::SUCCESS)
    {
        status = generate_velocity_trapezoidal(
            velocity_change, initial_acceleration, final_acceleration,
            -max_acceleration, -max_jerk, velocity_constraints, duration);
    }
    return status;
}

static TrapezoidalStatus generate_velocity_negative_change(
    const real_t velocity_change, const real_t initial_acceleration, const real_t final_acceleration,
    const real_t max_acceleration, const real_t max_jerk, TrapezoidalConstraints& velocity_constraints,
    TrapezoidalDuration& duration)
{
    TrapezoidalStatus status = generate_velocity_trapezoidal(
        velocity_change, initial_acceleration, final_acceleration,
        -max_acceleration, -max_jerk, velocity_constraints, duration);
    if (status != TrapezoidalStatus::SUCCESS)
    {
        status = generate_velocity_trapezoidal(
            velocity_change, initial_acceleration, final_acceleration,
            max_acceleration, max_jerk, velocity_constraints, duration);
    }
    return status;
}

// Trapezoidal Velocity
// ====================

TrapezoidalStatus TrapezoidalVelocity::goto_velocity(
    const real_t start_time, const TrajState& initial_state, const real_t final_velocity, const real_t final_acceleration,
    const real_t max_acceleration, const real_t max_jerk)
{
    auto status = TrapezoidalStatus::SUCCESS;
    if ((max_acceleration < FSB_TOL) ||
        (max_jerk < FSB_TOL))
    {
        status = TrapezoidalStatus::MAX_VALUE_BELOW_TOLERANCE;
    }
    else
    {
        TrapezoidalConstraints velocity_constraints = {};
        TrapezoidalDuration duration = {};
        if (const real_t velocity_change = final_velocity - initial_state.velocity;
            (fabs(initial_state.acceleration) < FSB_TOL) && (fabs(velocity_change) < FSB_TOL))
        {
            // no change in position
        }
        else
        {
            if (velocity_change >= 0.0)
            {
                status = generate_velocity_positive_change(velocity_change, initial_state.acceleration, final_acceleration,
                    max_acceleration, max_jerk, velocity_constraints, duration);
            }
            else
            {
                status = generate_velocity_negative_change(velocity_change, initial_state.acceleration, final_acceleration,
                    max_acceleration, max_jerk, velocity_constraints, duration);
            }
        }

        if (status == TrapezoidalStatus::SUCCESS)
        {
            m_start_time = start_time;
            m_total_duration = duration.start + duration.plateau + duration.end;

            m_seg1.generate(0.0, duration.start, initial_state, velocity_constraints.start_jerk);

            const TrajState seg2_initial_state = m_seg1.get_final_state();
            const real_t seg2_start_time = m_seg1.get_final_time();
            m_seg2.generate(seg2_start_time, duration.plateau, seg2_initial_state, velocity_constraints.plateau_acceleration);

            const TrajState seg3_initial_state = m_seg2.get_final_state();
            const real_t seg3_start_time = m_seg2.get_final_time();
            m_seg3.generate(seg3_start_time, duration.end, seg3_initial_state, velocity_constraints.end_jerk);

            const TrajState final_state = m_seg3.get_final_state();
            const real_t final_time = m_seg3.get_final_time();
            m_seg_extrapolate.generate(final_time, 0.0, final_state, final_state.acceleration);

            m_initial_state = initial_state;
            m_final_state = m_seg3.get_final_state();
        }
    }

    return status;
}

TrajState TrapezoidalVelocity::evaluate(real_t t_eval) const
{
    TrajState result = {};
    t_eval -= m_start_time;
    if (t_eval < 0.0)
    {
        result = m_initial_state;
    }
    else if (t_eval <= m_seg2.get_start_time())
    {
        // start ramp
        result = m_seg1.evaluate(t_eval);
    }
    else if (t_eval <= m_seg3.get_start_time())
    {
        // constant acceleration
        result = m_seg2.evaluate(t_eval);
    }
    else if (t_eval <= m_total_duration)
    {
        // end ramp
        result = m_seg3.evaluate(t_eval);
    }
    else
    {
        result = m_seg_extrapolate.evaluate(t_eval);
    }
    return result;
}

}
