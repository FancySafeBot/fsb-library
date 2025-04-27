
#include <doctest/doctest.h>
#include "fsb_test_macros.h"
#include "fsb_trapezoidal_velocity.h"

TEST_SUITE_BEGIN("trajectory_segment");

TEST_CASE("Smart pointer array of segments" * doctest::description("[fsb::Segment]"))
{
    constexpr size_t num_segments = 4U;
    fsb::SegmentConstJerk seg0 = {};
    fsb::SegmentConstAcc seg1 = {};
    fsb::SegmentConstJerk seg2 = {};
    fsb::SegmentConstVel seg3 = {};
    seg0.generate(0.0, 1.0,
        {1.0, 0.0, 0.0, 0.0}, 2.0);
    const auto seg1_init_state = seg0.get_final_state();
    seg1.generate(seg0.get_final_time(), 0.5, seg1_init_state, seg1_init_state.acceleration);
    const auto seg2_init_state = seg1.get_final_state();
    seg2.generate(seg1.get_final_time(), 1.0, seg2_init_state, -2.0);
    const auto seg3_init_state = seg2.get_final_state();
    seg3.generate(seg2.get_final_time(), 0.5, seg3_init_state, 0.0);

    // list of segments as trajectory
    std::array<std::unique_ptr<fsb::Segment>, num_segments> traj = {};
    traj[0] = std::make_unique<fsb::SegmentConstJerk>(seg0);
    traj[1] = std::make_unique<fsb::SegmentConstAcc>(seg1);
    traj[2] = std::make_unique<fsb::SegmentConstJerk>(seg2);
    traj[3] = std::make_unique<fsb::SegmentConstVel>(seg3);
}

TEST_SUITE_END();
