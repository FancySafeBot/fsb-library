
#include <ctime>
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_timing.h"

#include <iostream>

TEST_SUITE_BEGIN("timing");

TEST_CASE("Periodic timer 100 Hz" * doctest::description("[fsb::PeriodicTimer]"))
{
    // 10 ms timer
    const timespec step_size = {0, 10 * 1000 * 1000};
    const auto err_expected = fsb::TimingError::SUCCESS;

    // timing output
    fsb::TimeData nominal_time = {};
    fsb::TimeData actual_time = {};

    // run timer
    fsb::PeriodicTimer timer = {};
    const fsb::TimingError err_init = timer.initialize(step_size);
    const fsb::TimingError err_start = timer.start();
    const fsb::TimingError err_step = timer.step(nominal_time, actual_time);

    fsb::Real nominal_time_sec = fsb::timespec_to_seconds(nominal_time.monotonic);
    fsb::Real actual_time_sec =  fsb::timespec_to_seconds(actual_time.monotonic);

    // no error
    REQUIRE(err_expected == err_init);
    REQUIRE(err_expected == err_start);
    REQUIRE(err_expected == err_step);

    // check for positive non-zero timing results
    REQUIRE(nominal_time_sec > 0.0);
    REQUIRE(actual_time_sec > 0.0);

    // check elapsed time between target and next step
    const double step_size_sec = 1e-9 * static_cast<double>(step_size.tv_nsec);
    REQUIRE(nominal_time_sec == FsbApprox(step_size_sec));

    // check remainder
    REQUIRE((actual_time_sec - nominal_time_sec) < step_size_sec);
}

TEST_CASE("Periodic timer fail initialize with zero step size" * doctest::description("[fsb::PeriodicTimer]"))
{
    // 1 uss timer
    const timespec step_size = {0, 0};
    const auto err_expected = fsb::TimingError::STEP_SIZE_BELOW_CLOCK_RESOLUTION;
    fsb::PeriodicTimer timer = {};

    // Run timer
    const fsb::TimingError err_init = timer.initialize(step_size);
    // check
    REQUIRE(err_expected == err_init);
}

// static void print_readable_date(const timespec& ts) {
//     std::time_t t = ts.tv_sec;
//     char buf[64];
//     std::tm* tm_info = std::localtime(&t);
//     std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_info);
//     std::cout << buf << "." << ts.tv_nsec << std::endl;
// }
//
// TEST_CASE("Periodic timer test 1kHz" * doctest::description("[fsb::PeriodicTimer]"))
// {
//     /* set high priority FIFO scheduling */
//     sched_param pri_param;
//     int sched_policy;
//     int max_priority;
//     pthread_getschedparam (pthread_self (), &sched_policy, &pri_param);
//     max_priority = sched_get_priority_max (SCHED_FIFO);
//     pri_param.sched_priority = max_priority - 1;
//     pthread_setschedparam (pthread_self (), SCHED_FIFO, &pri_param);
//
//     pthread_getschedparam (pthread_self (), &sched_policy, &pri_param);
//     printf("\nSched policy is %s\n", (sched_policy == SCHED_FIFO ? "SCHED_FIFO" : "SCHED_OTHER"));
//     printf("Sched priority is %d/%d\n", pri_param.sched_priority, max_priority);
//
//     // 1 ms timer
//     const timespec step_size_1sec = {0, 100 * 1000};
//     const auto err_expected = fsb::TimingError::SUCCESS;
//     fsb::PeriodicTimer timer = {};
//
//     // Run timer
//     const fsb::TimingError err_init = timer.initialize(step_size_1sec);
//     const fsb::TimingError err_start = timer.start();
//
//     constexpr size_t num_steps = 10U;
//     std::array<fsb::TimeData, num_steps> nominal_time = {};
//     std::array<fsb::TimeData, num_steps> actual_time = {};
//     for (size_t i = 0; i < num_steps; ++i)
//     {
//         // timing output
//         if (const fsb::TimingError err_step = timer.step(nominal_time[i], actual_time[i]);
//             err_step != fsb::TimingError::SUCCESS)
//         {
//             std::cout << "Error!!";
//         }
//     }
//
//     for (size_t i = 0; i < num_steps; ++i)
//     {
//         // print result
//         fsb::real_t nominal_time_sec = fsb::timespec_to_seconds(actual_time[i].monotonic);
//         fsb::real_t remainder = nominal_time_sec - fsb::timespec_to_seconds(nominal_time[i].monotonic);
//         std::cout << i << " >> nominal_time sec: " << nominal_time_sec << std::endl;
//         std::cout << i << "    remainder sec: " << remainder << std::endl;
//         print_readable_date(actual_time[i].realtime);
//     }
//
//     REQUIRE(err_expected == err_init);
//     REQUIRE(err_expected == err_start);
//
// }

TEST_SUITE_END();
