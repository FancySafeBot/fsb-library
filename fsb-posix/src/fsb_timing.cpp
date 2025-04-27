#define _POSIX_C_SOURCE 200809L
#include <unistd.h>

#include <ctime>
#include "fsb_types.h"
#include "fsb_timing.h"

namespace fsb
{

// static methods
static real_t timespec_to_seconds(const timespec& ts);
static timespec timespec_monodiff(const timespec& ts_a, const timespec& ts_b);
static timespec timespec_monoadd(const timespec& ts_a, const timespec& ts_b);
static int clock_nanosleep_abstime (const timespec& req);

static real_t timespec_to_seconds(const timespec& ts)
{
    return static_cast<real_t>(ts.tv_sec) + static_cast<real_t>(ts.tv_nsec) * 1e-9;
}

/* timespec difference (monotonic) ts_a - ts_b */
static timespec timespec_monodiff(const timespec& ts_a, const timespec& ts_b)
{
    timespec ts_out = {
        ts_a.tv_sec - ts_b.tv_sec,
        ts_a.tv_nsec - ts_b.tv_nsec
    };

    // check for negative values
    if (ts_out.tv_sec < 0)
    {
        /* cannot be negative */
        ts_out.tv_sec = 0;
        ts_out.tv_nsec = 0;
    }
    else if (ts_out.tv_nsec < 0)
    {
        if (ts_out.tv_sec == 0)
        {
            /* cannot be negative */
            ts_out.tv_sec = 0;
            ts_out.tv_nsec = 0;
        }
        else
        {
            // offset seconds to keep nsec positive
            ts_out.tv_sec = ts_out.tv_sec - 1;
            ts_out.tv_nsec = ts_out.tv_nsec + 1000000000;
        }
    }
    else
    {
        /* nothing to alter */
    }

    return ts_out;
}

static timespec timespec_monoadd(const timespec& ts_a, const timespec& ts_b)
{
    timespec ts_out = {
        ts_a.tv_sec + ts_b.tv_sec,
        ts_a.tv_nsec + ts_b.tv_nsec
    };
    // keep nsec within interval [0, 1e9)
    if (ts_out.tv_nsec >= 1000000000)
    {
        ts_out.tv_sec = ts_out.tv_sec + 1;
        ts_out.tv_nsec = ts_out.tv_nsec - 1000000000;
    }

    return ts_out;
}

#ifdef FSB_NO_CLOCK_NANOSLEEP
/* emulate clock_nanosleep for CLOCK_MONOTONIC and TIMER_ABSTIME */
static int clock_nanosleep_abstime (const timespec& req)
{
    timespec ts_now = {};
    int retval = clock_gettime(CLOCK_MONOTONIC, &ts_now);
    if (retval == 0) {
        const timespec ts_req_relative = timespec_monodiff(req, ts_now);
        retval = nanosleep(&ts_req_relative, nullptr);
    }
    return retval;
}
#else
/* clock_nanosleep for CLOCK_MONOTONIC and TIMER_ABSTIME */
static int clock_nanosleep_abstime (const timespec& req)
{
    return clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &req, NULL);
}
#endif

TimingError PeriodicTimer::initialize(const timespec& step_size)
{
    auto err = TimingError::SUCCESS;

    // check clock resolution
    timespec res = {};
    if (0 != clock_getres(CLOCK_MONOTONIC, &res))
    {
        err = TimingError::MONOTONIC_CLOCK_FAILED;
    }
    else
    {
        if (const timespec res_test = timespec_monodiff(res, step_size);
            res_test.tv_sec > 0 || res_test.tv_nsec > 0)
        {
            err = TimingError::STEP_SIZE_BELOW_CLOCK_RESOLUTION;
        }
    }

    // check minimum step size
    if ((err == TimingError::SUCCESS) &&
        (step_size.tv_sec == 0) &&
        (step_size.tv_nsec < MINIMUM_STEP_SIZE_NS))
    {
        err = TimingError::STEP_SIZE_LESS_THAN_MINIMUM;
    }

    if (err == TimingError::SUCCESS)
    {
        m_step_size = step_size;
    }
    return err;
}

TimingError PeriodicTimer::start()
{
    auto err = TimingError::SUCCESS;
    if (0 != clock_gettime(CLOCK_MONOTONIC, &m_init))
    {
        err = TimingError::MONOTONIC_CLOCK_FAILED;
    }
    else
    {
        m_step_req = m_init;
    }
    return err;
}

TimingError PeriodicTimer::step(real_t& nominal_time, real_t& remainder)
{
    auto err = TimingError::SUCCESS;
    m_step_req = timespec_monoadd(m_step_req, m_step_size);
    if (0 != clock_nanosleep_abstime(m_step_req))
    {
        err = TimingError::SLEEP_FAILED;
    }
    else
    {
        timespec now = {};
        if (0 != clock_gettime(CLOCK_MONOTONIC, &now))
        {
            err = TimingError::MONOTONIC_CLOCK_FAILED;
        }
        else
        {
            /* elapsed from last step */
            remainder = timespec_to_seconds(timespec_monodiff(now, m_step_req));
            /* time from start in seconds */
            nominal_time = timespec_to_seconds(timespec_monodiff(m_step_req, m_init));
        }
    }

    return err;
}

}
