#ifndef FSB_PID_H
#define FSB_PID_H

#include "fsb_types.h"

#include <cstdint>

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
        real_t step_size, real_t gain_kp, real_t gain_ki, real_t gain_kd, real_t filter_tf,
        PidType type = PidType::TRAPEZOIDAL);

    /**
     * @brief Reset PID to given command and measure
     *
     * The integral and derivative components of the internal state will be set to 0
     *
     * @param command Command value at reset
     * @param measured Measured value at reset
     */
    void reset(real_t command, real_t measured);

    /**
     * @brief Evalaute PID
     *
     * @param command Input command
     * @param measured Measured feedback
     * @return Controller output
     */
    real_t evaluate(real_t command, real_t measured);

private:
    /** proportional gain */
    real_t m_kp = 0.0;
    /** integral gain */
    real_t m_ki = 0.0;
    /** derivative gain */
    real_t m_kd = 0.0;
    /** time constant for derivative filter (in seconds) */
    real_t m_tf = 0.0;
    /** Step size in seconds */
    real_t m_step_size = 0.0;
    /** PID type */
    PidType m_type = PidType::TRAPEZOIDAL;

    /** Controller state */
    real_t m_e1 = 0.0;
    real_t m_e2 = 0.0;
    real_t m_u1 = 0.0;
    real_t m_u2 = 0.0;
};

/**
 * @}
 */

} // namespace fsb

#endif
