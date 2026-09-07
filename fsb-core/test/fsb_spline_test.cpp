
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

        const fsb::SplineError bad_step = traj.generate(0.0, std::array<fsb::Real, 2U>{0.0, 1.0});
        const fsb::SplineError not_enough_points = traj.generate(1.0, std::array<fsb::Real, 1U>{0.0});
        const fsb::SplineError too_many_points =
            traj.generate(1.0, std::array<fsb::Real, 9U>{});

        REQUIRE(bad_step == fsb::SplineError::INVALID_STEP_SIZE);
        REQUIRE(not_enough_points == fsb::SplineError::NOT_ENOUGH_POINTS);
        REQUIRE(too_many_points == fsb::SplineError::TOO_MANY_POINTS);
    }

    SUBCASE("linear two-point spline has constant velocity")
    {
        fsb::Spline<8U> traj = {};
        REQUIRE(traj.generate(0.5, std::array<fsb::Real, 2U>{0.0, 1.0}) == fsb::SplineError::SUCCESS);

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
        REQUIRE(traj.generate(1.0, std::array<fsb::Real, 4U>{0.0, 1.0, 0.0, 1.0}) == fsb::SplineError::SUCCESS);

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

    SUBCASE("clamped spline has zero endpoint velocity")
    {
        fsb::Spline<6U> spl = {};
        const std::array<fsb::Real, 6U> points = {0.0, 1.2, 0.2, 1.1, -0.3, 0.0};

        REQUIRE(spl.generate(0.1, points, fsb::SplineEndCondition::Clamped) == fsb::SplineError::SUCCESS);

        const fsb::TrajState initial_state = spl.get_initial_state();
        const fsb::TrajState final_state = spl.get_final_state();
        REQUIRE(initial_state.velocity == FsbApprox(0.0, 1e-8));
        REQUIRE(final_state.velocity == FsbApprox(0.0, 1e-8));
    }

    SUBCASE("clamped and natural produce different endpoint velocity")
    {
        const std::array<fsb::Real, 4U> points = {0.0, 1.0, 0.0, 1.0};

        fsb::Spline<8U> natural = {};
        fsb::Spline<8U> clamped = {};

        REQUIRE(natural.generate(1.0, points, fsb::SplineEndCondition::Natural) == fsb::SplineError::SUCCESS);
        REQUIRE(clamped.generate(1.0, points, fsb::SplineEndCondition::Clamped) == fsb::SplineError::SUCCESS);

        const fsb::TrajState natural_initial = natural.get_initial_state();
        const fsb::TrajState clamped_initial = clamped.get_initial_state();
        REQUIRE(clamped_initial.velocity == FsbApprox(0.0, 1e-8));
        REQUIRE(natural_initial.velocity != FsbApprox(0.0, 1e-8));

    }
    SUBCASE("failed updates preserve generated spline")
    {
        fsb::Spline<4U> traj = {};
        REQUIRE(traj.generate(1.0, std::array<fsb::Real, 2U>{0.0, 1.0}) == fsb::SplineError::SUCCESS);

        const fsb::TrajState expected_final_state = traj.get_final_state();
        const fsb::Real expected_duration = traj.get_duration();

        REQUIRE(traj.generate(0.0, std::array<fsb::Real, 2U>{2.0, 3.0})
            == fsb::SplineError::INVALID_STEP_SIZE);

        REQUIRE(traj.get_duration() == FsbApprox(expected_duration));
        REQUIRE(traj.get_final_state().position == FsbApprox(expected_final_state.position));
        REQUIRE(traj.get_final_state().velocity == FsbApprox(expected_final_state.velocity));
    }

}

} // TEST_SUITE
