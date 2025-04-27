#pragma once

#include "fsb_types.h"
#include <ctime>

namespace fsb
{

/**
 * @defgroup TopicTiming Timer management
 * @brief Timed Loop Management
 * @{
 */

/**
* Minimum step size in nanoseconds
*/
constexpr long MINIMUM_STEP_SIZE_NS = 10000;

/**
 * @brief Timing error codes
 */
enum class TimingError
{
    /**
     * @brief No error
     */
    SUCCESS,
    /**
     * @brief Failed to get monotonic clock time
     */
    MONOTONIC_CLOCK_FAILED,
    /**
     * Failed nanosleep
     */
    SLEEP_FAILED,
    /**
     * @brief Step size must be specified above clock resolution
     */
    STEP_SIZE_BELOW_CLOCK_RESOLUTION,
    /**
     * @brief Step size must be specified greater than minimum
     */
    STEP_SIZE_LESS_THAN_MINIMUM
};

/**
 * @brief Compute forward kinematics
 */
class PeriodicTimer
{
public:
    PeriodicTimer() = default;

    /**
     * @brief Initialize periodic timer
     *
     * @param step_size Timer step size
     * @return Timing error code
     */
    TimingError initialize(const timespec& step_size);

    /**
     * @brief Start timer.
     *
     * The current time from the high-resolution monotonic timer will be used.
     *
     * @return Timing error code
     */
    TimingError start();

    /**
     * @brief Wait for next step
     *
     * The target time is based on number of times @c step is called after
     * @c calling start. The actual elapsed time is equal to the addition of the
     * nominal time and remainder.
     *
     * @param nominal_time Target elapsed time since calling @c start()
     * @param remainder Actual time delay after target elapsed time
     * @return Timing error code
     */
    TimingError step(real_t& nominal_time, real_t& remainder);

private:

    /** @brief Next requested time to wake after sleeping in timestep */
    timespec m_step_req = {};
    /** @brief Initial monotonic time when starting loop */
    timespec m_init = {};
    /** @brief Step size of timer loop */
    timespec m_step_size = {};
};


/**
 * @}
 */

} // namespace fsb
