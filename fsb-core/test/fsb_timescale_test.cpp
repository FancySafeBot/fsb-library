
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_timescale.h"

TEST_SUITE_BEGIN("timescale");

TEST_CASE("Apply timescale" * doctest::description("[fsb::timescale_trajectory]"))
{
    const fsb::TrajState time_state = {
    0.0, 4.0, 6.0, 0.0};
    const fsb::TrajState traj_state = {
    1.0, 2.0, 3.0, 0.0};

    const fsb::TrajState traj_scaled_expected = {
    1.0, 8.0, 60.0, 0.0};

    const fsb::TrajState traj_scaled_actual = fsb::timescale_trajectory(time_state, traj_state);

    REQUIRE(traj_scaled_actual.position == FsbApprox(traj_scaled_expected.position));
    REQUIRE(traj_scaled_actual.velocity == FsbApprox(traj_scaled_expected.velocity));
    REQUIRE(traj_scaled_actual.acceleration == FsbApprox(traj_scaled_expected.acceleration));
    REQUIRE(traj_scaled_actual.jerk == FsbApprox(traj_scaled_expected.jerk));
}

TEST_CASE("Evaluate timescale trajectory" * doctest::description("[fsb::Timescale]"))
{
    // input limits
    const fsb::real_t max_timescale = 2.0;
    const fsb::real_t max_timescale_deriv = 10.0;
    const fsb::real_t max_timescale_2nd_deriv = 100.0;
    // input initial values
    const fsb::real_t time_mono_start = 0.0;
    const fsb::real_t time_scaled_start = 1.0;
    const fsb::real_t timescale_start = 1.5;
    // input set timescale
    const fsb::real_t time_mono_set = 0.0;
    const fsb::real_t timescale_set = 1.0;
    // input evaluation time
    const fsb::real_t time_mono_eval = 0.141;

    // Expected
    // https://www.trajectorygenerator.com/ojet-online/?xs0=1.0&xt0=0&xs1=1.5&xt1=1.0&xs2=0&xt2=0&xp0=1000&xn0=-1000&xp1=2.0&xp2=10&xp3=100&mode=1&tvec=nt&ntdt=10000#start
    const fsb::TrajState time_state_expected = {
    1.1763553378125,
    1.0000088770539,
    -0.042135623730949,
    100.0
    };
    const auto expected_result = fsb::TimescaleResult::SUCCESS;

    // Process
    fsb::Timescale timescale = {};
    const bool set_limits_success = timescale.set_limits(max_timescale, max_timescale_deriv, max_timescale_2nd_deriv);
    timescale.start(time_mono_start, time_scaled_start, timescale_start);
    const fsb::TimescaleResult set_timescale_result = timescale.goto_timescale(time_mono_set, timescale_set);
    const fsb::TrajState time_state_actual = timescale.evaluate(time_mono_eval);

    // Check output
    REQUIRE(set_limits_success == true);
    REQUIRE(set_timescale_result == expected_result);
    REQUIRE(time_state_actual.position == FsbApprox(time_state_expected.position));
    REQUIRE(time_state_actual.velocity == FsbApprox(time_state_expected.velocity));
    REQUIRE(time_state_actual.acceleration == FsbApprox(time_state_expected.acceleration));
    REQUIRE(time_state_actual.jerk == FsbApprox(time_state_expected.jerk));
}

TEST_CASE("Timescale small limits" * doctest::description("[fsb::Timescale]"))
{
    // input limits
    fsb::real_t max_timescale = -FSB_TOL;
    fsb::real_t max_timescale_deriv = 10.0;
    fsb::real_t max_timescale_2nd_deriv = 100.0;
    // Expected
    const bool set_limits_success_expected = false;

    // Process
    fsb::Timescale timescale = {};
    bool set_limits_success = timescale.set_limits(max_timescale, max_timescale_deriv, max_timescale_2nd_deriv);
    REQUIRE(set_limits_success_expected == set_limits_success);

    max_timescale = 2.0;
    max_timescale_deriv = 0.5 * FSB_TOL;
    set_limits_success = timescale.set_limits(max_timescale, max_timescale_deriv, max_timescale_2nd_deriv);
    REQUIRE(set_limits_success_expected == set_limits_success);

    max_timescale_deriv = 10.0;
    max_timescale_2nd_deriv = 0.5 * FSB_TOL;
    set_limits_success = timescale.set_limits(max_timescale, max_timescale_deriv, max_timescale_2nd_deriv);
    REQUIRE(set_limits_success_expected == set_limits_success);

    max_timescale_2nd_deriv = 100.0;
    set_limits_success = timescale.set_limits(max_timescale, max_timescale_deriv, max_timescale_2nd_deriv);
    REQUIRE(true == set_limits_success);
}

TEST_SUITE_END();
