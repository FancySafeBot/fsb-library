#ifndef FSB_THREAD_H
#define FSB_THREAD_H

#include <cstdlib>
#include <cstdint>
#include <pthread.h>
#include <sys/types.h>

namespace fsb
{

/**
 * @defgroup   Thread Posix Threads
 * @brief      Threading for Posix systems
 *
 * @{
 *
 */

/**
 * @brief Status codes for thread operations
 */
enum class ThreadStatus : uint8_t
{
    SUCCESS,     ///< Operation completed successfully
    ERROR,       ///< Operation failed with an error
    UNSUPPORTED  ///< Operation is not supported on this platform
};

/**
 * @brief Status codes for lock and synchronization operations
 */
enum class LockStatus : uint8_t
{
    SUCCESS,  ///< Operation completed successfully
    TIMEOUT,  ///< Operation timed out
    ERROR     ///< Operation failed with an error
};

/**
 * @brief Set the scheduling priority of a thread
 *
 * @param thread The pthread handle of the thread to modify
 * @param policy The scheduling policy (e.g., SCHED_FIFO, SCHED_RR, SCHED_OTHER)
 * @param priority The priority value within the policy's range
 * @return ThreadStatus indicating success or failure
 */
ThreadStatus set_thread_priority(pthread_t thread, int policy, int priority);
/**
 * @brief Set the CPU affinity of a thread to a specific CPU core
 *
 * @param thread The pthread handle of the thread to modify
 * @param cpu_index The index of the CPU core to bind the thread to
 * @return ThreadStatus indicating success or failure
 */
ThreadStatus set_thread_cpu_affinity(pthread_t thread, size_t cpu_index);

/**
 * @brief Initialize a mutex
 *
 * @param mutex Reference to the mutex to initialize
 * @param shared If true, mutex can be shared across processes; if false, only within threads
 * @return LockStatus indicating success or failure
 */
LockStatus mutex_initialize(pthread_mutex_t& mutex, bool shared = false);
/**
 * @brief Lock a mutex, blocking until the lock is acquired
 *
 * @param mutex Reference to the mutex to lock
 * @return LockStatus indicating success or failure
 */
LockStatus mutex_lock(pthread_mutex_t& mutex);
/**
 * @brief Unlock a mutex
 *
 * @param mutex Reference to the mutex to unlock
 * @return LockStatus indicating success or failure
 */
LockStatus mutex_unlock(pthread_mutex_t& mutex);
/**
 * @brief Destroy a mutex and release its resources
 *
 * @param mutex Reference to the mutex to destroy
 * @return LockStatus indicating success or failure
 */
LockStatus mutex_destroy(pthread_mutex_t& mutex);

/**
 * @brief Initialize a condition variable
 *
 * @param cond_var Reference to the condition variable to initialize
 * @param shared If true, condition variable can be shared across processes; if false, only within threads
 * @return LockStatus indicating success or failure
 */
LockStatus condvar_initialize(pthread_cond_t& cond_var, bool shared = false);
/**
 * @brief Wait on a condition variable with a timeout
 *
 * @param cond_var Reference to the condition variable to wait on
 * @param mutex Reference to the mutex associated with the condition variable
 * @param timeout Absolute time at which to timeout
 * @return LockStatus indicating success, timeout, or failure
 */
LockStatus condvar_wait_timeout(
    pthread_cond_t& cond_var, pthread_mutex_t& mutex, const struct timespec& timeout);
/**
 * @brief Signal one or all threads waiting on a condition variable
 *
 * @param cond_var Reference to the condition variable to signal
 * @param broadcast If true, wake all waiting threads; if false, wake only one thread
 * @return LockStatus indicating success or failure
 */
LockStatus condvar_signal(pthread_cond_t& cond_var, bool broadcast = false);
/**
 * @brief Destroy a condition variable and release its resources
 *
 * @param cond_var Reference to the condition variable to destroy
 * @return LockStatus indicating success or failure
 */
LockStatus condvar_destroy(pthread_cond_t& cond_var);

/*
 * @}
 */
} // namespace fsb

#endif
