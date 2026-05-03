
#include <array>
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_spline.h"

TEST_SUITE("spline") {

TEST_CASE("Spline trajectory" * doctest::description("[fsb::SplineTrajectory]"))
{
    SUBCASE("generate rejects invalid inputs")
    {
        fsb::Spline<8U> traj = {};

        const bool bad_step = traj.generate(0.0, std::array<fsb::Real, 2U>{0.0, 1.0});
        const bool not_enough_points = traj.generate(1.0, std::array<fsb::Real, 1U>{0.0});

        REQUIRE(bad_step == false);
        REQUIRE(not_enough_points == false);
    }

    SUBCASE("linear two-point spline has constant velocity")
    {
        fsb::Spline<8U> traj = {};
        REQUIRE(traj.generate(0.5, std::array<fsb::Real, 2U>{0.0, 1.0}));

        REQUIRE(traj.get_start_time() == FsbApprox(0.0));
        REQUIRE(traj.get_duration() == FsbApprox(0.5));
        REQUIRE(traj.get_final_time() == FsbApprox(0.5));

        const fsb::TrajState s0 = traj.evaluate(0.0);
        const fsb::TrajState sm = traj.evaluate(0.25);
        const fsb::TrajState s1 = traj.evaluate(0.5);

        REQUIRE(s0.position == FsbApprox(0.0));
        REQUIRE(s0.velocity == FsbApprox(2.0));
        REQUIRE(s0.acceleration == FsbApprox(0.0));
        REQUIRE(s0.jerk == FsbApprox(0.0));

        REQUIRE(sm.position == FsbApprox(0.5));
        REQUIRE(sm.velocity == FsbApprox(2.0));
        REQUIRE(sm.acceleration == FsbApprox(0.0));
        REQUIRE(sm.jerk == FsbApprox(0.0));

        REQUIRE(s1.position == FsbApprox(1.0));
        REQUIRE(s1.velocity == FsbApprox(2.0));
        REQUIRE(s1.acceleration == FsbApprox(0.0));
        REQUIRE(s1.jerk == FsbApprox(0.0));
    }

    SUBCASE("natural spline has zero endpoint acceleration and clamps evaluation time")
    {
        fsb::Spline<8U> traj = {};
        REQUIRE(traj.generate(1.0, std::array<fsb::Real, 4U>{0.0, 1.0, 0.0, 1.0}));

        const fsb::TrajState initial_state = traj.get_initial_state();
        const fsb::TrajState final_state = traj.get_final_state();
        REQUIRE(initial_state.acceleration == FsbApprox(0.0, 1e-9));
        REQUIRE(final_state.acceleration == FsbApprox(0.0, 1e-9));

        const fsb::TrajState below_start = traj.evaluate(-10.0);
        const fsb::TrajState at_start = traj.evaluate(traj.get_start_time());
        REQUIRE(below_start.position == FsbApprox(at_start.position));
        REQUIRE(below_start.velocity == FsbApprox(at_start.velocity));

        const fsb::TrajState above_end = traj.evaluate(100.0);
        const fsb::TrajState at_end = traj.evaluate(traj.get_final_time());
        REQUIRE(above_end.position == FsbApprox(at_end.position));
        REQUIRE(above_end.velocity == FsbApprox(at_end.velocity));
    }

    SUBCASE("append extends spline and updates duration")
    {
        fsb::Spline<8U> traj = {};
        REQUIRE(traj.generate(1.0, std::array<fsb::Real, 2U>{0.0, 1.0}));
        REQUIRE(traj.append(std::array<fsb::Real, 2U>{2.0, 3.0}));

        REQUIRE(traj.get_duration() == FsbApprox(3.0));
        REQUIRE(traj.get_final_time() == FsbApprox(3.0));

        const fsb::TrajState end_state = traj.evaluate(3.0);
        REQUIRE(end_state.position == FsbApprox(3.0));
    }

}

} // TEST_SUITE
