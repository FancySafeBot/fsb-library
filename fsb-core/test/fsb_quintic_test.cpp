
#include <array>
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_quintic.h"

TEST_SUITE_BEGIN("quintic");

TEST_CASE("Quintic trajectory" * doctest::description("[fsb::QuinticTrajectory]"))
{
    // Inputs
    const fsb::TrajState initial_state = {0.0, 0.0, 0.0, 0.0};
    const fsb::TrajState final_state = {10.0, 5.0, -3.0, 0.0};
    const fsb::real_t start_time = 1.0;
    const fsb::real_t duration = 10.0;
    const size_t num_pnts = 2;
    const std::array<fsb::real_t, num_pnts> input_time = {
        1.0, 11.0
    };
    // Expected
    const bool gen_success_expected = true;
    const std::array<bool, num_pnts> out_of_range_expected = {
        false,
        false
    };
    const std::array<fsb::TrajState, num_pnts> output_expected = {
        fsb::TrajState{0.0, 0.0, 0.0, 0.0},
        fsb::TrajState{10.0, 5.0, -3.0, 0.0}
    };
    // Generate
    fsb::QuinticTrajectory traj = {};
    const bool gen_success_actual = traj.generate(start_time, duration, initial_state, final_state);
    // Evaluate
    std::array<fsb::TrajState, num_pnts> output_actual = {};
    std::array<bool, num_pnts> out_of_range_actual = {};
    for (size_t k = 0; k < num_pnts; ++k) {
        output_actual[k] = traj.evaluate(input_time[k]);
    }
    // Test
    REQUIRE(gen_success_actual == gen_success_expected);
    for (size_t k = 0; k < num_pnts; ++k)
    {
        REQUIRE(out_of_range_actual[k] == out_of_range_expected[k]);
        REQUIRE(output_expected[k].position == FsbApprox(output_actual[k].position));
        REQUIRE(output_expected[k].velocity == FsbApprox(output_actual[k].velocity));
        REQUIRE(output_expected[k].acceleration == FsbApprox(output_actual[k].acceleration));
    }
}

TEST_SUITE_END();
