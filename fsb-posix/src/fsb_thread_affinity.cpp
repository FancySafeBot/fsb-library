#define _POSIX_C_SOURCE 200809L
#include <unistd.h>
#include <limits.h>
#include <errno.h>

#include "fsb_thread.h"

namespace fsb
{

static int get_num_cpus()
{
    const int num_cpus = sysconf(_SC_NPROCESSORS_ONLN);
    return num_cpus;
}

ThreadStatus set_thread_cpu_affinity(const pthread_t thread, const size_t cpu_index)
{
    // Get the number of CPUs available on the system
    const int num_cpus = sysconf(_SC_NPROCESSORS_ONLN);
    if (num_cpus <= 0 || cpu_index >= static_cast<size_t>(num_cpus)) {
        return ThreadStatus::ERROR;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(static_cast<int>(cpu_index), &cpuset);

    const int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        return ThreadStatus::ERROR;
    }

    return ThreadStatus::SUCCESS;
}

}