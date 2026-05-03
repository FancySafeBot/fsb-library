
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_trapezoidal_velocity.h"
#include "fsb_trapezoidal_position.h"

TEST_SUITE("trapezoidal") {

TEST_CASE("Zero duration trajectory" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        10.0, 50.0, 0.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 50.0;
    const fsb::Real final_acceleration = 0.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.0;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 4;
    const fsb::Real input_time[num_input_values] = {
        -1e-6, 0.0, 1e-6, 1.0
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 10.0, 50.0, 0.0, 0.0 },
        { 10.0, 50.0, 0.0, 0.0 },
        { 10.00005, 50.0, 0.0, 0.0 },
        { 60.0, 50.0, 0.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 0" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, 0.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 50.0;
    const fsb::Real final_acceleration = 0.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.4472135954999579;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.044721359549995794, -8.94427190999916e-06, 0.0, 8.94427190999916e-06, 0.22360679774997896, 0.4472046512280479, 0.4472135954999579, 0.4472225397718679, 0.49193495504995377
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, 0.0, 0.0 },
        { 0.0, 0.0, 0.0, 0.0 },
        { 0.0, 0.0, 0.0, 1000.0 },
        { 1.192569587999888e-13, 4.000000000000001e-08, 0.00894427190999916, 1000.0 },
        { 1.8633899812498245, 25.0, 223.60679774997897, 1000.0 },
        { 11.179892673903566, 49.99999996, 0.00894427190999636, -1000.0 },
        { 11.180339887498949, 50.0, 0.0, -1000.0 },
        { 11.180787101094447, 50.0, 0.0, 0.0 },
        { 13.416407864998742, 50.0, 0.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 1" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, 0.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 0.1;
    const fsb::Real final_acceleration = 0.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.02;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.002, -4.0000000000000003e-07, 0.0, 4.0000000000000003e-07, 0.01, 0.0199996, 0.02, 0.020000399999999998, 0.022000000000000002
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, 0.0, 0.0 },
        { 0.0, 0.0, 0.0, 0.0 },
        { 0.0, 0.0, 0.0, 1000.0 },
        { 1.066666666666667e-17, 8.000000000000001e-11, 0.0004, 1000.0 },
        { 0.00016666666666666666, 0.05, 10.0, 1000.0 },
        { 0.0009999600000000107, 0.09999999992, 0.0004000000000008441, -1000.0 },
        { 0.0009999999999999998, 0.10000000000000002, 0.0, -1000.0 },
        { 0.0010000399999999996, 0.10000000000000002, 0.0, 0.0 },
        { 0.0012000000000000001, 0.10000000000000002, 0.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 2" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, 0.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 20.0;
    const fsb::Real final_acceleration = 0.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.282842712474619;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.0282842712474619, -5.656854249492381e-06, 0.0, 5.656854249492381e-06, 0.1414213562373095, 0.2828370556203695, 0.282842712474619, 0.2828483693288685, 0.3111269837220809
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 1000.0 }, { 3.016988933062603e-14, 1.6000000000000004e-08, 0.005656854249492381, 1000.0 }, { 0.4714045207910316, 10.0, 141.4213562373095, 1000.0 }, { 2.8283139876612293, 19.999999984, 0.005656854249536991, -1000.0 }, { 2.8284271247461894, 20.0, 0.0, -1000.0 }, { 2.828540261831179, 20.0, 0.0, 0.0 }, { 3.3941125496954276, 20.0, 0.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 3" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, 20.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 50.0;
    const fsb::Real final_acceleration = -5.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.43316291680593116;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.043316291680593116, -8.663258336118624e-06, 0.0, 8.663258336118624e-06, 0.21658145840296558, 0.433154253547595, 0.43316291680593116, 0.43317158006426726, 0.4764792084865243
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, 20.0, 0.0 }, { 0.0, 0.0, 20.0, 0.0 }, { 0.0, 0.0, 20.0, 1000.0 }, { 7.506288158590341e-10, 0.00017330269274487165, 20.008663258336117, 1000.0 }, { 2.1616410481002895, 27.62914323003707, 211.58145840296558, -1000.0 }, { 11.414332816926727, 50.00004327876565, -4.991336741663872, -1000.0 }, { 11.414765980031056, 50.0, -5.0, -1000.0 }, { 11.415199142760232, 49.99995668370832, -5.0, 0.0 }, { 13.575889811248317, 49.78341854159704, -5.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 4" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, 20.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 0.1;
    const fsb::Real final_acceleration = -10.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.03449489742783178;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.0034494897427831783, -6.898979485566357e-07, 0.0, 6.898979485566357e-07, 0.01724744871391589, 0.034494207529883227, 0.03449489742783178, 0.034495587325780334, 0.03794438717061496
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, 20.0, 0.0 }, { 0.0, 0.0, 20.0, 0.0 }, { 0.0, 0.0, 20.0, -1000.0 }, { 4.759537067016297e-12, 1.3797720991543004e-05, 19.999310102051442, -1000.0 }, { 0.00211963221128641, 0.19621173070873837, 2.7525512860841097, -1000.0 }, { 0.005061793186304266, 0.10000689921746513, -10.000689897948554, 1000.0 }, { 0.005061862178478972, 0.1, -10.0, 1000.0 }, { 0.005061931165894032, 0.09999310102051447, -10.0, 0.0 }, { 0.0053473162553294585, 0.06550510257216818, -10.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 5" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, 20.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 20.0;
    const fsb::Real final_acceleration = -10.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.2746049894151541;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.02746049894151541, -5.492099788303083e-06, 0.0, 5.492099788303083e-06, 0.13730249470757705, 0.2745994973153658, 0.2746049894151541, 0.2746104815149424, 0.3020654883566695
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, 20.0, 0.0 }, { 0.0, 0.0, 20.0, 0.0 }, { 0.0, 0.0, 20.0, 1000.0 }, { 3.0165921069430694e-10, 0.000109857077346104, 20.005492099788302, 1000.0 }, { 0.618798618017826, 11.947037420613654, 127.30249470757708, -1000.0 }, { 3.02759467779885, 20.0000549059163, -9.994507900211687, -1000.0 }, { 3.027704519945404, 20.0, -9.999999999999972, -1000.0 }, { 3.0278143617903543, 19.999945079002117, -10.0, 0.0 }, { 3.5731441037651277, 19.725395010584847, -10.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 6" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, -20.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 50.0;
    const fsb::Real final_acceleration = 10.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.45833023542919793;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.04583302354291979, -9.166604708583959e-06, 0.0, 9.166604708583959e-06, 0.22916511771459896, 0.45832106882448936, 0.45833023542919793, 0.4583394020339065, 0.5041632589721178
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, -20.0, 0.0 }, { 0.0, 0.0, -20.0, 0.0 }, { 0.0, 0.0, -20.0, 1000.0 }, { -8.401380456658136e-10, -0.00018329008085073747, -19.990833395291418, 1000.0 }, { 1.4806642463930726, 21.67502323428102, 209.16511771459898, 1000.0 }, { 10.671172788257595, 49.999908291939605, 10.009166604708525, -1000.0 }, { 10.67163111807276, 50.0, 9.999999999999972, -1000.0 }, { 10.672089448728324, 50.00009166604708, 10.0, 0.0 }, { 12.973785625454186, 50.4583302354292, 10.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 7" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, -20.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 0.1;
    const fsb::Real final_acceleration = 10.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.04741657386773941;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.004741657386773942, -9.483314773547884e-07, 0.0, 9.483314773547884e-07, 0.023708286933869706, 0.04741562553626206, 0.04741657386773941, 0.047417522199216765, 0.052158231254513354
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, -20.0, 0.0 }, { 0.0, 0.0, -20.0, 0.0 }, { 0.0, 0.0, -20.0, 1000.0 }, { -8.993183765185054e-12, -1.8966179880800296e-05, -19.999051668522647, 1000.0 }, { -0.0033998250355711996, -0.1931243040080456, 3.7082869338697044, 1000.0 }, { -0.004935509175344416, 0.09999051623556016, 10.000948331477355, -1000.0 }, { -0.0049354143466934855, 0.09999999999999998, 10.000000000000002, -1000.0 }, { -0.004935319509049088, 0.1000094833147735, 10.0, 0.0 }, { -0.004348832034148352, 0.14741657386773938, 10.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

TEST_CASE("Velocity Case 8" * doctest::description("[fsb::VelocityTrapezoidal]"))
{
    // floating point tolerance
    const fsb::Real tolerance = 1e-10;

    // Inputs
    // ------------------------

    // constraints
    const fsb::Real max_acceleration = 300.0;
    const fsb::Real max_jerk  = 1000.0;
    // start time
    const fsb::Real start_time = 0.0;
    // initial state
    const fsb::TrajState initial_state = {
        0.0, 0.0, -20.0, 0.0
    };
    // target
    const fsb::Real final_velocity = 20.0;
    const fsb::Real final_acceleration = 10.0;

    // Expected output
    // --------------------------

    // SetTargetPosition duration
    const fsb::Real expected_duration = 0.2946049894151541;
    // Evaluate inputs and expected values
    constexpr size_t num_input_values = 9;
    const fsb::Real input_time[num_input_values] = {
        -0.029460498941515412, -5.892099788303083e-06, 0.0, 5.892099788303083e-06, 0.14730249470757706, 0.2945990973153658, 0.2946049894151541, 0.2946108815149424, 0.32406548835666954
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0, 0.0, -20.0, 0.0 }, { 0.0, 0.0, -20.0, 0.0 }, { 0.0, 0.0, -20.0, 1000.0 }, { -3.4713430663902645e-10, -0.000117824637346104, -19.9941079002117, 1000.0 }, { 0.31571528468449284, 7.902962579386344, 127.30249470757707, 1000.0 }, { 2.6215866781232573, 19.999941061643696, 10.005892099788326, -1000.0 }, { 2.6217045199454043, 20.0, 10.000000000000028, -1000.0 }, { 2.6218223621147545, 20.000058920997883, 10.0, 0.0 }, { 3.215254103765128, 20.294604989415156, 10.0, 0.0 }
    };

    // Run trapezoidal test trajectory
    // -------------------------------

    // trapezoidal object
    fsb::TrapezoidalVelocity traj = {};
    // Generate trajectory
    fsb::TrapezoidalStatus set_target_status = traj.goto_velocity(start_time, initial_state, final_velocity, final_acceleration,
                                                                  max_acceleration, max_jerk);
    // Test result
    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == set_target_status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));
    // Evaluate
    for (size_t k = 0; k < num_input_values; ++k) {
        fsb::TrajState actual_output = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position == FsbApprox(actual_output.position, tolerance));
        REQUIRE(expected_output[k].velocity == FsbApprox(actual_output.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual_output.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk == FsbApprox(actual_output.jerk, tolerance));
    }
}

} // TEST_SUITE

TEST_SUITE("trapezoidal_position") {

TEST_CASE("Position invalid inputs" * doctest::description("[fsb::TrapezoidalPosition]"))
{
    fsb::TrapezoidalPosition traj = {};

    // zero max velocity
    REQUIRE(fsb::TrapezoidalStatus::MAX_VALUE_BELOW_TOLERANCE ==
            traj.generate(0.0, 0.0, 1.0, 0.0, 10.0, 100.0));
    // zero max acceleration
    REQUIRE(fsb::TrapezoidalStatus::MAX_VALUE_BELOW_TOLERANCE ==
            traj.generate(0.0, 0.0, 1.0, 5.0, 0.0, 100.0));
    // zero max jerk
    REQUIRE(fsb::TrapezoidalStatus::MAX_VALUE_BELOW_TOLERANCE ==
            traj.generate(0.0, 0.0, 1.0, 5.0, 10.0, 0.0));
}

TEST_CASE("Position zero displacement" * doctest::description("[fsb::TrapezoidalPosition]"))
{
    const fsb::Real tolerance = 1e-10;

    // Inputs
    const fsb::Real max_velocity = 5.0;
    const fsb::Real max_acceleration = 10.0;
    const fsb::Real max_jerk = 100.0;
    const fsb::Real start_time = 1.5;
    const fsb::Real initial_position = 3.0;
    const fsb::Real final_position = 3.0;

    const fsb::Real expected_duration = 0.0;
    constexpr size_t num_input_values = 3;
    const fsb::Real input_time[num_input_values] = { 0.0, 1.5, 3.0 };
    const fsb::TrajState expected_output[num_input_values] = {
        { 3.0, 0.0, 0.0, 0.0 },
        { 3.0, 0.0, 0.0, 0.0 },
        { 3.0, 0.0, 0.0, 0.0 }
    };

    fsb::TrapezoidalPosition traj = {};
    const fsb::TrapezoidalStatus status = traj.generate(
        start_time, initial_position, final_position,
        max_velocity, max_acceleration, max_jerk);

    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration()));

    for (size_t k = 0; k < num_input_values; ++k)
    {
        const fsb::TrajState actual = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position     == FsbApprox(actual.position, tolerance));
        REQUIRE(expected_output[k].velocity     == FsbApprox(actual.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk         == FsbApprox(actual.jerk, tolerance));
    }
}

TEST_CASE("Position forward with cruise" * doctest::description("[fsb::TrapezoidalPosition]"))
{
    // p0=0, pf=10, vmax=5, amax=10, jmax=100
    // v_thresh = amax^2/jmax = 1.0
    // d_accel(vmax) = 5^2/(2*10) + 5*10/(2*100) = 1.25 + 0.25 = 1.5
    // d_no_cruise = 3.0 < 10.0  => cruise phase
    // t_ramp = vmax/amax + amax/jmax = 0.6
    // t_cruise = (10 - 3) / 5 = 1.4
    // total duration = 2*0.6 + 1.4 = 2.6
    // Timeline: [0, 0.6] accel, [0.6, 2.0] cruise, [2.0, 2.6] decel

    const fsb::Real tolerance = 1e-10;

    // Inputs
    const fsb::Real max_velocity = 5.0;
    const fsb::Real max_acceleration = 10.0;
    const fsb::Real max_jerk = 100.0;
    const fsb::Real start_time = 0.0;
    const fsb::Real initial_position = 0.0;
    const fsb::Real final_position = 10.0;

    const fsb::Real expected_duration = 2.6;
    constexpr size_t num_input_values = 8;
    const fsb::Real input_time[num_input_values] = {
        -0.5,    // before start: extrapolate backwards (clamp at initial)
         0.0,    // start
         0.1,    // inside accel ramp phase 1 (constant jerk up), end of jerk-up phase
         0.6,    // end of accel ramp
         1.3,    // mid-cruise
         2.0,    // end of cruise / start of decel ramp
         2.6,    // end of decel ramp
         3.5     // after end: extrapolate forwards (clamp at final)
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0,                  0.0, 0.0,   0.0  },  // before start (clamp to initial state, j=0)
        { 0.0,                  0.0, 0.0, 100.0  },  // t=0.0: jerk begins
        { 1.0/60.0,             0.5, 10.0, 100.0 },  // t=0.1: end of jerk-up phase 1
        { 1.5,                  5.0, 0.0,  -100.0 },  // t=0.6: end of accel ramp
        { 5.0,                  5.0, 0.0,   0.0  },  // t=1.3: mid-cruise
        { 8.5,                  5.0, 0.0,   0.0  },  // t=2.0: end of cruise
        { 10.0,                 0.0, 0.0,   0.0  },  // t=2.6: end of trajectory
        { 10.0,                 0.0, 0.0,   0.0  }   // after end: clamped
    };

    fsb::TrapezoidalPosition traj = {};
    const fsb::TrapezoidalStatus status = traj.generate(
        start_time, initial_position, final_position,
        max_velocity, max_acceleration, max_jerk);

    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration(), tolerance));

    for (size_t k = 0; k < num_input_values; ++k)
    {
        const fsb::TrajState actual = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position     == FsbApprox(actual.position, tolerance));
        REQUIRE(expected_output[k].velocity     == FsbApprox(actual.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk         == FsbApprox(actual.jerk, tolerance));
    }
}

TEST_CASE("Position no cruise plateau accel" * doctest::description("[fsb::TrapezoidalPosition]"))
{
    // p0=0, pf=2, vmax=5, amax=10, jmax=100
    // v_thresh = 1.0, d_no_cruise(vmax=5) = 3.0 > 2.0  => no cruise
    // Plateau solve: v^2 + v*1.0 - 2*10 = 0  =>  v = (-1 + 9)/2 = 4.0
    // d_accel(4) = 16/20 + 4*10/200 = 0.8 + 0.2 = 1.0  => 2*1.0 = 2.0 OK
    // t_ramp = 4/10 + 10/100 = 0.5, total = 1.0
    // Timeline: [0, 0.5] accel, [0.5, 1.0] decel

    const fsb::Real tolerance = 1e-10;

    const fsb::Real max_velocity = 5.0;
    const fsb::Real max_acceleration = 10.0;
    const fsb::Real max_jerk = 100.0;
    const fsb::Real start_time = 0.0;
    const fsb::Real initial_position = 0.0;
    const fsb::Real final_position = 2.0;

    const fsb::Real expected_duration = 1.0;
    constexpr size_t num_input_values = 5;
    // t=0.1: end of accel jerk-up: p=1/60, v=0.5, a=10
    // t=0.4 (end of const-accel phase): p=37/60, v=3.5, a=10, j=0 (segment boundary)
    // t=0.5: end of accel ramp: p=1.0, v=4.0, a=0
    // t=1.0: end of decel ramp: p=2.0, v=0.0, a=0.0, j=100 (decel ramp jerk-up final segment)
    const fsb::Real input_time[num_input_values] = {
        0.0, 0.1, 0.4, 0.5, 1.0
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0,                0.0, 0.0,  100.0 },
        { 1.0/60.0,           0.5, 10.0, 100.0 },
        { 37.0/60.0,          3.5, 10.0,   0.0 },
        { 1.0,                4.0, 0.0, -100.0 },
        { 2.0,                0.0, 0.0,  100.0 }
    };

    fsb::TrapezoidalPosition traj = {};
    const fsb::TrapezoidalStatus status = traj.generate(
        start_time, initial_position, final_position,
        max_velocity, max_acceleration, max_jerk);

    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration(), tolerance));

    for (size_t k = 0; k < num_input_values; ++k)
    {
        const fsb::TrajState actual = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position     == FsbApprox(actual.position, tolerance));
        REQUIRE(expected_output[k].velocity     == FsbApprox(actual.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk         == FsbApprox(actual.jerk, tolerance));
    }
}

TEST_CASE("Position no cruise triangle accel" * doctest::description("[fsb::TrapezoidalPosition]"))
{
    // p0=0, pf=0.1, vmax=5, amax=10, jmax=100
    // v_thresh = 1.0
    // Triangle: v_peak = (0.05 * sqrt(100))^(2/3) = (0.05*10)^(2/3) = 0.5^(2/3) = 0.6299605...
    // v_peak < v_thresh => confirmed triangle
    // t_j = sqrt(v_peak/jmax) = 0.07937..., total = 4*t_j = 0.31748..., no cruise, no const-accel

    const fsb::Real tolerance = 1e-9;

    const fsb::Real max_velocity = 5.0;
    const fsb::Real max_acceleration = 10.0;
    const fsb::Real max_jerk = 100.0;
    const fsb::Real start_time = 0.0;
    const fsb::Real initial_position = 0.0;
    const fsb::Real final_position = 0.1;

    // v_peak = (0.05 * sqrt(100))^(2/3)
    const fsb::Real v_peak = 0.62996052494743658;
    const fsb::Real t_j = 0.079370052598409974;
    const fsb::Real expected_duration = 4.0 * t_j;

    // At t=2*t_j: peak of velocity profile (midpoint)
    //   Position = 0.05 (half of total displacement)
    //   Velocity = v_peak
    //   Acceleration = 0 (transition between jerk-up and jerk-down of each ramp)
    constexpr size_t num_input_values = 4;
    const fsb::Real input_time[num_input_values] = {
        0.0, t_j, 2.0*t_j, 4.0*t_j
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0,    0.0,    0.0,    100.0 },
        { 100.0*t_j*t_j*t_j/6.0, 100.0*t_j*t_j/2.0, 100.0*t_j, 100.0 },
        { 0.05,   v_peak, 0.0,   -100.0  },
        { 0.1,    0.0,    0.0,   100.0   }
    };

    fsb::TrapezoidalPosition traj = {};
    const fsb::TrapezoidalStatus status = traj.generate(
        start_time, initial_position, final_position,
        max_velocity, max_acceleration, max_jerk);

    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration(), tolerance));

    for (size_t k = 0; k < num_input_values; ++k)
    {
        const fsb::TrajState actual = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position     == FsbApprox(actual.position, tolerance));
        REQUIRE(expected_output[k].velocity     == FsbApprox(actual.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk         == FsbApprox(actual.jerk, tolerance));
    }
}

TEST_CASE("Position negative motion" * doctest::description("[fsb::TrapezoidalPosition]"))
{
    // p0=5, pf=0, vmax=5, amax=10, jmax=100
    // Symmetric to fwd_cruise with displacement=-5 instead of +10
    // d_no_cruise(vmax=5) = 3.0 < 5.0 => has cruise
    // t_cruise = (5-3)/5 = 0.4, total = 2*0.6 + 0.4 = 1.6
    // velocity is negative: -5.0 during cruise

    const fsb::Real tolerance = 1e-10;

    const fsb::Real max_velocity = 5.0;
    const fsb::Real max_acceleration = 10.0;
    const fsb::Real max_jerk = 100.0;
    const fsb::Real start_time = 0.0;
    const fsb::Real initial_position = 5.0;
    const fsb::Real final_position = 0.0;

    const fsb::Real expected_duration = 1.6;
    constexpr size_t num_input_values = 5;
    const fsb::Real input_time[num_input_values] = {
        0.0, 0.6, 1.0, 1.6, 2.0
    };
    // At t=0.6: end of accel ramp, p = 5.0 - 1.5 = 3.5, v = -5
    // At t=1.0: mid-cruise, p = 3.5 + (-5)*0.4 = 1.5, v = -5
    // At t=1.6: end, p = 0.0, v = 0.0
    const fsb::TrajState expected_output[num_input_values] = {
        { 5.0,  0.0,  0.0, -100.0  },
        { 3.5, -5.0,  0.0,  100.0  },
        { 1.5, -5.0,  0.0,  0.0  },
        { 0.0,  0.0,  0.0,  0.0  },
        { 0.0,  0.0,  0.0,  0.0  }
    };

    fsb::TrapezoidalPosition traj = {};
    const fsb::TrapezoidalStatus status = traj.generate(
        start_time, initial_position, final_position,
        max_velocity, max_acceleration, max_jerk);

    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration(), tolerance));

    for (size_t k = 0; k < num_input_values; ++k)
    {
        const fsb::TrajState actual = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position     == FsbApprox(actual.position, tolerance));
        REQUIRE(expected_output[k].velocity     == FsbApprox(actual.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk         == FsbApprox(actual.jerk, tolerance));
    }
}

TEST_CASE("Position non-zero start time" * doctest::description("[fsb::TrapezoidalPosition]"))
{
    // Same as fwd_cruise but with start_time=2.0
    // p0=0, pf=10, vmax=5, amax=10, jmax=100, t_start=2.0
    // Timeline: [2.0, 2.6] accel, [2.6, 4.0] cruise, [4.0, 4.6] decel
    // total duration = 2.6 (same as t_start=0 case)

    const fsb::Real tolerance = 1e-10;

    const fsb::Real max_velocity = 5.0;
    const fsb::Real max_acceleration = 10.0;
    const fsb::Real max_jerk = 100.0;
    const fsb::Real start_time = 2.0;
    const fsb::Real initial_position = 0.0;
    const fsb::Real final_position = 10.0;

    const fsb::Real expected_duration = 2.6;
    constexpr size_t num_input_values = 5;
    const fsb::Real input_time[num_input_values] = {
        0.0, 2.0, 2.6, 4.0, 4.6
    };
    const fsb::TrajState expected_output[num_input_values] = {
        { 0.0,  0.0, 0.0, 0.0 },  // before start: clamped (initial state, j=0)
        { 0.0,  0.0, 0.0, 100.0 },// t=start
        { 1.5,  5.0, 0.0, 0.0 },  // t=2.6: end of accel ramp
        { 8.5,  5.0, 0.0, 0.0 },  // t=4.0: end of cruise
        { 10.0, 0.0, 0.0, 100.0 } // t=4.6: end (decel ramp final jerk-up segment)
    };

    fsb::TrapezoidalPosition traj = {};
    const fsb::TrapezoidalStatus status = traj.generate(
        start_time, initial_position, final_position,
        max_velocity, max_acceleration, max_jerk);

    REQUIRE(fsb::TrapezoidalStatus::SUCCESS == status);
    REQUIRE(expected_duration == FsbApprox(traj.get_duration(), tolerance));

    for (size_t k = 0; k < num_input_values; ++k)
    {
        const fsb::TrajState actual = traj.evaluate(input_time[k]);
        REQUIRE(expected_output[k].position     == FsbApprox(actual.position, tolerance));
        REQUIRE(expected_output[k].velocity     == FsbApprox(actual.velocity, tolerance));
        REQUIRE(expected_output[k].acceleration == FsbApprox(actual.acceleration, tolerance));
        REQUIRE(expected_output[k].jerk         == FsbApprox(actual.jerk, tolerance));
    }
}

} // TEST_SUITE
