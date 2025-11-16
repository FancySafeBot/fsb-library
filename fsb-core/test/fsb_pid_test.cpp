
#include <array>
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_pid.h"

TEST_SUITE_BEGIN("pid");

TEST_CASE("Trapezoidal Sample PID" * doctest::description("[fsb::PidController]"))
{
    const fsb::Real tolerance = 1e-10;
    // Inputs
    const auto pid_type = fsb::PidType::TRAPEZOIDAL;
    const fsb::Real step_size = 0.05;
    const fsb::Real gain_kp = 2.221;
    const fsb::Real gain_ki = 0.835;
    const fsb::Real gain_kd = 1.462;
    const fsb::Real filter_tf = 0.010348;
    constexpr size_t num_steps = 5;
    const std::array<fsb::Real, num_steps> command = {0.0, 1.0, 1.0, 1.0, 1.0};
    const std::array<fsb::Real, num_steps> measured = {0.0, 0.0, 0.000874312243086, 0.00556889735979, 0.0133625159264};
    // expected output
    const std::array<fsb::Real, num_steps> output_expected = {0.0, 43.6020651098, -14.8985928216, 9.24002507103, -0.85680893914};
    // run process
    std::array<fsb::Real, num_steps> output_actual = {};
    fsb::PidController controller = {};
    controller.initialize(step_size, gain_kp, gain_ki, gain_kd, filter_tf, pid_type);
    for (size_t k = 0; k < num_steps; ++k)
    {
        output_actual[k] = controller.evaluate(command[k], measured[k]);
    }
    // check result
    for (size_t k = 0; k < num_steps; ++k)
    {
        REQUIRE(output_actual[k] == FsbApprox(output_expected[k], tolerance));
    }
}

TEST_CASE("Backwards Sample PID" * doctest::description("[fsb::PidController]"))
{
    const fsb::Real tolerance = 1e-10;
    // Inputs
    const auto pid_type = fsb::PidType::BACKWARDS;
    const fsb::Real step_size = 0.05;
    const fsb::Real gain_kp = 2.2;
    const fsb::Real gain_ki = 0.856;
    const fsb::Real gain_kd = 1.283;
    const fsb::Real filter_tf = 0.0005;
    constexpr size_t num_steps = 5;
    const std::array<fsb::Real, num_steps> command = {0.0, 1.0, 1.0, 1.0, 1.0};
    const std::array<fsb::Real, num_steps> measured = {0.0, 0.0, 0.000554414850454, 0.00377133177074, 0.0100115470657};
    // expected output
    const std::array<fsb::Real, num_steps> output_expected = {0.0, 27.6487405941, 2.5218150939, 2.24054020259, 2.18923650313};
    // run process
    std::array<fsb::Real, num_steps> output_actual = {};
    fsb::PidController controller = {};
    controller.initialize(step_size, gain_kp, gain_ki, gain_kd, filter_tf, pid_type);
    for (size_t k = 0; k < num_steps; ++k)
    {
        output_actual[k] = controller.evaluate(command[k], measured[k]);
    }
    // check result
    for (size_t k = 0; k < num_steps; ++k)
    {
        REQUIRE(output_actual[k] == FsbApprox(output_expected[k], tolerance));
    }
}

TEST_SUITE_END();
