
#include "fsb_thread.h"

namespace fsb
{

ThreadStatus set_thread_cpu_affinity(const pthread_t /* thread */, const size_t /* cpu_index */)
{
    return ThreadStatus::UNSUPPORTED;
}

}