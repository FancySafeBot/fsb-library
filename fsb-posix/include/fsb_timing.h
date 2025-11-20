#ifndef FSB_TIMING_H
#define FSB_TIMING_H

#include <cstdint>
#include <ctime>

#include "fsb_types.h"

namespace fsb
{

/**
 * @defgroup TopicTiming Timer management
 * @brief Timed Loop Management
 * @{
 */

/**
 * @brief Minimum step size in nanoseconds
 */
constexpr long kMinimumStepSizeNs = 10000;

/**
 * @brief Convert timespec to seconds as a Real value
 *
 * @param ts The timespec structure to convert
 * @return Real The time in seconds
 */
inline Real timespec_to_seconds(const timespec& ts)
{
    return static_cast<Real>(ts.tv_sec) + static_cast<Real>(ts.tv_nsec) * 1e-9;
}

/**
 * @brief Timing error codes
 */
enum class TimingError : uint8_t
{
    /**
     * @brief No error
     */
    SUCCESS = 0,
    /**
     * @brief Failed to get monotonic clock time
     */
    MONOTONIC_CLOCK_FAILED = 1,
    /**
     * @brief Failed to get realtime clock time
     */
    REALTIME_CLOCK_FAILED = 2,
    /**
     * @brief Failed nanosleep
     */
    SLEEP_FAILED = 3,
    /**
     * @brief Step size must be specified above clock resolution
     */
    STEP_SIZE_BELOW_CLOCK_RESOLUTION = 4,
    /**
     * @brief Step size must be specified greater than minimum
     */
    STEP_SIZE_LESS_THAN_MINIMUM = 5
};

struct TimeData
{
    /** @brief Monotonic elapsed time */
    struct timespec monotonic = {};
    /** @brief Epoch time */
    struct timespec realtime = {};
};

/**
 * @brief Periodic timer
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
     * @param nominal_time Target time since calling \c start().
     * @param actual_time Actual time.
     * @return Timing error code
     */
    TimingError step(TimeData& nominal_time, TimeData& actual_time);

private:
    /** @brief Next requested time to wake after sleeping in timestep */
    timespec m_step_req = {};
    /** @brief Initial monotonic time when starting loop */
    timespec m_init = {};
    /** @brief Initial realtime clock time when starting loop */
    timespec m_init_realtime = {};
    /** @brief Step size of timer loop */
    timespec m_step_size = {};
};

/**
 * @}
 */

} // namespace fsb

#endif
