# headers and source files
set(FSBCORE_TEST_HEADERS
    fsb_test_macros.h
    fsb_body_tree_sample.h)
set(FSBCORE_TEST_SOURCES
    fsb_quaternion_test.cpp
    fsb_rotation_test.cpp
    fsb_motion_test.cpp
    fsb_joint_test.cpp
    fsb_body_test.cpp
    fsb_body_tree_test.cpp
    fsb_kinematics_test.cpp
    fsb_jacobian_test.cpp
    fsb_body_tree_sample.cpp
    fsb_linalg_test.cpp
    fsb_linalg3_test.cpp
    fsb_interface_test.cpp
    fsb_trapezoidal_test.cpp
    fsb_test_main.cpp
    fsb_pid_test.cpp
    fsb_quintic_test.cpp
    fsb_timescale_test.cpp
    fsb_jacobian_test.cpp
    fsb_spatial_test.cpp
    fsb_trajectory_segment_test.cpp
    fsb_dynamics_test.cpp)

add_executable(fsbcore_test
    ${FSBCORE_TEST_HEADERS}
    ${FSBCORE_TEST_SOURCES})

# dependency: doctest
CPMAddPackage(
    NAME doctest
    GITHUB_REPOSITORY onqtam/doctest
    VERSION ${DOCTEST_VERSION}
    OPTIONS
    "BUILD_SHARED_LIBS OFF"
    "BUILD_TESTING OFF"
    "CMAKE_POLICY_VERSION_MINIMUM 3.5"
)

target_link_libraries(fsbcore_test PRIVATE
    doctest
    FancySafeBot::fsbcore)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(fsbcore_test)
