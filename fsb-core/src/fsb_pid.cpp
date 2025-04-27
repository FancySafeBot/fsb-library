
#include "fsb_types.h"
#include "fsb_pid.h"

namespace fsb
{

static real_t pid_run_back(
    const real_t err, const real_t step_size, const real_t gain_kp, const real_t gain_ki,
    const real_t gain_kd, const real_t filter_tf, const real_t state_e1, const real_t state_e2, const real_t state_u1,
    const real_t state_u2)
{
    /* calculate output */
    return (err * gain_kd - state_e1 * gain_kd * 2.0 + state_e2 * gain_kd + filter_tf * state_u1 * 2.0
           - filter_tf * state_u2 + step_size * state_u1 + err * gain_ki * (step_size * step_size)
           + err * gain_kp * filter_tf + err * gain_kp * step_size - state_e1 * gain_kp * filter_tf * 2.0
           + state_e2 * gain_kp * filter_tf - state_e1 * gain_kp * step_size + err * gain_ki * filter_tf * step_size
           - state_e1 * gain_ki * filter_tf * step_size)
          / (filter_tf + step_size);
}

static real_t pid_run_trap(
    const real_t err, const real_t step_size, const real_t gain_kp, const real_t gain_ki,
    const real_t gain_kd, const real_t filter_tf, const real_t state_e1, const real_t state_e2, const real_t state_u1,
    const real_t state_u2)
{
    /* calculate output */
    const real_t step_size_2 = step_size * step_size;
    return (err * gain_kp * step_size - state_e2 * gain_kp * step_size + err * gain_kp * filter_tf * 2.0
           - state_e1 * gain_kp * filter_tf * 4.0 + state_e2 * gain_kp * filter_tf * 2.0
           + err * gain_ki * step_size_2 * (1.0 / 2.0) + state_e1 * gain_ki * step_size_2
           + state_e2 * gain_ki * step_size_2 * (1.0 / 2.0) + err * gain_kd * 2.0 - state_e1 * gain_kd * 4.0
           + state_e2 * gain_kd * 2.0 + filter_tf * state_u1 * 4.0 - filter_tf * state_u2 * 2.0 + step_size * state_u2
           + err * gain_ki * filter_tf * step_size - state_e2 * gain_ki * filter_tf * step_size)
          / (filter_tf * 2.0 + step_size);
}

void PidController::initialize(
    const real_t step_size, const real_t gain_kp, const real_t gain_ki, const real_t gain_kd, const real_t filter_tf,
    const PidType type)
{
    m_step_size = step_size;
    m_kp = gain_kp;
    m_ki = gain_ki;
    m_kd = gain_kd;
    m_tf = filter_tf;
    m_type = type;

    m_e1 = 0.0;
    m_e2 = 0.0;
    m_u1 = 0.0;
    m_u2 = 0.0;
}

void PidController::reset(const real_t command, const real_t measured)
{
    // assume steady-state with no change in error
    const real_t err = command - measured;
    const real_t output = m_kp * err;
    m_e1 = err;
    m_e2 = err;
    m_u1 = output;
    m_u2 = output;
}

real_t PidController::evaluate(const real_t command, const real_t measured)
{
    const real_t err = command - measured;
    real_t output = 0.0;
    if (m_type == PidType::BACKWARDS)
    {
        output = pid_run_back(err, m_step_size, m_kp, m_ki, m_kd, m_tf,
                            m_e1, m_e2, m_u1, m_u2);
    }
    else
    {
        output = pid_run_trap(err, m_step_size, m_kp, m_ki, m_kd, m_tf,
                            m_e1, m_e2, m_u1, m_u2);
    }
    /* advance state */
    m_e2 = m_e1;
    m_e1 = err;
    m_u2 = m_u1;
    m_u1 = output;

    return output;
}

} // namespace fsb
