#define _POSIX_C_SOURCE 200809L
#include <unistd.h>
#include <limits.h>
#include <errno.h>

#include "fsb_thread.h"

namespace fsb
{

ThreadStatus set_thread_priority(pthread_t thread, int policy, int priority)
{
    sched_param sch_params;
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(thread, policy, &sch_params) != 0) {
        return ThreadStatus::ERROR;
    }
    return ThreadStatus::SUCCESS;
}

LockStatus mutex_initialize(pthread_mutex_t& mutex, const bool shared)
{
    // mutex
    pthread_mutexattr_t m_attr;
    if (pthread_mutexattr_init(&m_attr) != 0)
    {
        return LockStatus::ERROR;
    }
    if (shared && pthread_mutexattr_setpshared(&m_attr, PTHREAD_PROCESS_SHARED) != 0)
    {
        pthread_mutexattr_destroy(&m_attr);
        return LockStatus::ERROR;
    }
    if (pthread_mutex_init(&mutex, &m_attr) != 0)
    {
        pthread_mutexattr_destroy(&m_attr);
        return LockStatus::ERROR;
    }

    // cleanup
    pthread_mutexattr_destroy(&m_attr);

    return LockStatus::SUCCESS;
}

LockStatus mutex_lock(pthread_mutex_t& mutex)
{
    if (pthread_mutex_lock(&mutex) != 0) {
        return LockStatus::ERROR;
    }
    return LockStatus::SUCCESS;
}

LockStatus mutex_unlock(pthread_mutex_t& mutex)
{
    if (pthread_mutex_unlock(&mutex) != 0) {
        return LockStatus::ERROR;
    }
    return LockStatus::SUCCESS;
}

LockStatus mutex_destroy(pthread_mutex_t& mutex)
{
    if (pthread_mutex_destroy(&mutex) != 0) {
        return LockStatus::ERROR;
    }
    return LockStatus::SUCCESS;
}

LockStatus condvar_initialize(pthread_cond_t& cond_var, const bool shared)
{
    // condvar
    pthread_condattr_t cv_attr;
    if (pthread_condattr_init(&cv_attr) != 0)
    {
        return LockStatus::ERROR;
    }
#if defined(__APPLE__)
    // MacOS does not support CLOCK_MONOTONIC for condition variables
#else
    if (pthread_condattr_setclock(&cv_attr, CLOCK_MONOTONIC) != 0)
    {
        return LockStatus::ERROR;
    }
#endif
    if (shared && pthread_condattr_setpshared(&cv_attr, PTHREAD_PROCESS_SHARED) != 0)
    {
        pthread_condattr_destroy(&cv_attr);
        return LockStatus::ERROR;
    }
    if (pthread_cond_init(&cond_var, &cv_attr) != 0)
    {
        pthread_condattr_destroy(&cv_attr);
        return LockStatus::ERROR;
    }

    pthread_condattr_destroy(&cv_attr);
    return LockStatus::SUCCESS;
}

LockStatus condvar_wait_timeout(pthread_cond_t& cond_var, pthread_mutex_t& mutex, const struct timespec& timeout)
{
    const int retval = pthread_cond_timedwait(&cond_var, &mutex, &timeout);
    if (retval != 0) {
        return (retval == ETIMEDOUT ? LockStatus::TIMEOUT : LockStatus::ERROR);
    }
    return LockStatus::SUCCESS;
}

LockStatus condvar_signal(pthread_cond_t& cond_var, const bool broadcast)
{
    if (broadcast) {
        if (pthread_cond_broadcast(&cond_var) != 0) {
            return LockStatus::ERROR;
        }
    } else {
        if (pthread_cond_signal(&cond_var) != 0) {
            return LockStatus::ERROR;
        }
    }
    return LockStatus::SUCCESS;
}

LockStatus condvar_destroy(pthread_cond_t& cond_var)
{
    if (pthread_cond_destroy(&cond_var) != 0) {
        return LockStatus::ERROR;
    }
    return LockStatus::SUCCESS;
}

}
