#pragma once

#include <cstdlib>
#include <cstdint>
#include <pthread.h>
#include <array>

#include "fsb_configuration.h"

namespace fsb
{

/**
 * @defgroup   Thread Posix Threads
 * @brief      Threading for Posix systems
 *
 * @{
 *
 */

 enum class ThreadStatus
 {
    SUCCESS,
    ERROR,
    UNSUPPORTED
};

enum class LockStatus
{
    SUCCESS,
    TIMEOUT,
    ERROR
};

ThreadStatus set_thread_priority(pthread_t thread, int policy, int priority);
ThreadStatus set_thread_cpu_affinity(pthread_t thread, size_t cpu_index);

LockStatus mutex_initialize(pthread_mutex_t& mutex, bool shared = false);
LockStatus mutex_lock(pthread_mutex_t& mutex);
LockStatus mutex_unlock(pthread_mutex_t& mutex);
LockStatus mutex_destroy(pthread_mutex_t& mutex);

LockStatus condvar_initialize(pthread_cond_t& cond_var, bool shared = false);
LockStatus condvar_wait_timeout(pthread_cond_t& cond_var, pthread_mutex_t& mutex, const struct timespec& timeout);
LockStatus condvar_signal(pthread_cond_t& cond_var, bool broadcast = false);
LockStatus condvar_destroy(pthread_cond_t& cond_var);

 /*
  * @}
  */
}
