#ifndef FSB_PID_H
#define FSB_PID_H

#include <cstdint>
#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup TopicPid PID Controller
 * @brief Proportial Integral Derivaitve controller
 * @{
 */

/**
 * @brief Type of discretization
 */
enum class PidType : uint8_t
{
    /**
     * @brief Trapezoidal discretization
     */
    TRAPEZOIDAL = 0,
    /**
     * @brief Backwards difference discretization
     *
     */
    BACKWARDS = 1
};

/**
 * @brief PID Controller
 */
class PidController
{
public:
    PidController() = default;

    /**
     * @brief Initialize PID
     *
     * @param step_size Step size in seconds
     * @param gain_kp Proportional gain
     * @param gain_ki  Integral gain
     * @param gain_kd  Derivative gain
     * @param filter_tf Derivative filter time constant
     * @param type Type of discretization
     */
    void initialize(
        Real step_size, Real gain_kp, Real gain_ki, Real gain_kd, Real filter_tf,
        PidType type = PidType::TRAPEZOIDAL);

    /**
     * @brief Reset PID to given command and measure
     *
     * The integral and derivative components of the internal state will be set to 0
     *
     * @param command Command value at reset
     * @param measured Measured value at reset
     */
    void reset(Real command, Real measured);

    /**
     * @brief Evalaute PID
     *
     * @param command Input command
     * @param measured Measured feedback
     * @return Controller output
     */
    Real evaluate(Real command, Real measured);

private:
    /** proportional gain */
    Real m_kp = 0.0;
    /** integral gain */
    Real m_ki = 0.0;
    /** derivative gain */
    Real m_kd = 0.0;
    /** time constant for derivative filter (in seconds) */
    Real m_tf = 0.0;
    /** Step size in seconds */
    Real m_step_size = 0.0;
    /** PID type */
    PidType m_type = PidType::TRAPEZOIDAL;

    /** Controller state */
    Real m_e1 = 0.0;
    Real m_e2 = 0.0;
    Real m_u1 = 0.0;
    Real m_u2 = 0.0;
};

/**
 * @}
 */

} // namespace fsb

#endif
