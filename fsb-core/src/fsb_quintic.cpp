
#include "fsb_quintic.h"

#include "fsb_types.h"
#include "fsb_trajectory_types.h"

namespace fsb
{

static QuinticCoeffs generate_coefficients(
    const real_t duration, const TrajState& initial_state, const TrajState& final_state,
    bool& is_valid)
{
    QuinticCoeffs coeffs = {};
    is_valid = (duration > QUINTIC_MIN_DURATION);
    if (is_valid)
    {
        const real_t dist = final_state.position - initial_state.position;
        const real_t tf2 = duration * duration;
        const real_t tf3 = tf2 * duration;
        const real_t tf4 = tf3 * duration;
        const real_t tf5 = tf4 * duration;

        coeffs.c0 = initial_state.position;
        coeffs.c1 = initial_state.velocity;
        coeffs.c2 = 0.5 * initial_state.acceleration;
        coeffs.c3 = (1.0 / (2.0 * tf3))
                    * (20.0 * dist
                       - (8.0 * final_state.velocity + 12.0 * initial_state.velocity) * duration
                       - (3.0 * initial_state.acceleration - final_state.acceleration) * tf2);
        coeffs.c4 = (1.0 / (2.0 * tf4))
                    * (-30.0 * dist
                       + (14.0 * final_state.velocity + 16.0 * initial_state.velocity) * duration
                       + (3.0 * initial_state.acceleration - 2.0 * final_state.acceleration) * tf2);
        coeffs.c5
            = (1.0 / (2.0 * tf5))
              * (12.0 * dist - 6.0 * (final_state.velocity + initial_state.velocity) * duration
                 + (final_state.acceleration - initial_state.acceleration) * tf2);
    }

    return coeffs;
}

// def generate_symmetric(self, initial: TrajectoryPva, final: TrajectoryPva, step_size=None):
//     """[summary]
//
//     Args:
//         initial (TrajectoryPva): [description]
//         final (TrajectoryPva): [description]
//
//     Returns:
//         [type]: [description]
//     """
//     h = final.position - initial.position
//     vel_sum = final.velocity + initial.velocity
//     if np.fabs(h) < EPS:
//         return _error_value['BADINPUT']
//     if np.fabs(vel_sum) < EPS:
//         return _error_value['BADINPUT']
//     if (h / vel_sum) < EPS:
//         return _error_value['BADINPUT']
//     if np.fabs(initial.acceleration) > EPS:
//         return _error_value['BADINPUT']
//     if np.fabs(final.acceleration) > EPS:
//         return _error_value['BADINPUT']
//
//     if step_size is not None:
//         if step_size < EPS:
//             return _error_value['BADINPUT']
//
//         num_steps = int(np.ceil((2.0 * h / vel_sum) / step_size))
//         duration = float(num_steps) * step_size
//         return self.generate(duration, initial, final)
//
//     duration = 2.0 * h / vel_sum
//     if duration > self.max_duration:
//         return _error_value['MAX_DURATION_EXCEEDED']
//
//     vel_diff = final.velocity - initial.velocity
//     vel_sqr = vel_sum * vel_sum
//     h_sqr = h * h
//
//     self.coeffs = [
//         0.0,
//         -(1.0 / 16.0) * vel_diff * vel_sqr * vel_sum / (h_sqr * h),
//         (1.0 / 4.0) * vel_diff * vel_sqr / h_sqr,
//         0,
//         initial.velocity,
//         initial.position
//     ]
//

bool QuinticTrajectory::generate(
    const real_t start_time, const real_t duration, const TrajState& initial_state,
    const TrajState& final_state)
{
    bool is_valid = false;

    const QuinticCoeffs coeffs
        = generate_coefficients(duration, initial_state, final_state, is_valid);
    if (is_valid)
    {
        m_coeffs = coeffs;
        m_start_time = start_time;
        m_duration = duration;
    }
    return is_valid;
}

TrajState QuinticTrajectory::evaluate(real_t t_eval) const
{
    TrajState result = {};
    t_eval -= m_start_time;

    const real_t x2 = t_eval * t_eval;
    const real_t x3 = x2 * t_eval;
    const real_t x4 = x3 * t_eval;
    const real_t x5 = x4 * t_eval;

    result.jerk = m_coeffs.c5 * 60.0 * x2 + m_coeffs.c4 * 24.0 * t_eval + m_coeffs.c3 * 6.0;

    result.acceleration = m_coeffs.c5 * 20.0 * x3 + m_coeffs.c4 * 12.0 * x2
                          + m_coeffs.c3 * 6.0 * t_eval + m_coeffs.c2 * 2.0;

    result.velocity = m_coeffs.c5 * 5.0 * x4 + m_coeffs.c4 * 4.0 * x3 + m_coeffs.c3 * 3.0 * x2
                      + m_coeffs.c2 * 2.0 * t_eval + m_coeffs.c1;

    result.position = m_coeffs.c5 * x5 + m_coeffs.c4 * x4 + m_coeffs.c3 * x3 + m_coeffs.c2 * x2
                      + m_coeffs.c1 * t_eval + m_coeffs.c0;

    return result;
}

} // namespace fsb
