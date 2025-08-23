
# headers and source files
set(FSBURDF_TEST_HEADERS
    fsb_test_macros.h)
set(FSBURDF_TEST_SOURCES
    fsb_urdf_test_main.cpp
    fsb_urdf_utilities_test.cpp
    fsb_urdf_origin_test.cpp
    fsb_urdf_inertial_test.cpp
    fsb_urdf_body_test.cpp
    fsb_urdf_joint_test.cpp
    fsb_urdf_test.cpp)

add_executable(fsburdf_test
    ${FSBURDF_TEST_HEADERS}
    ${FSBURDF_TEST_SOURCES})

# Copy test data
file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# dependency: doctest
CPMAddPackage(
    NAME doctest
    GITHUB_REPOSITORY onqtam/doctest
    VERSION ${DOCTEST_VERSION}
    OPTIONS
    "BUILD_SHARED_LIBS OFF"
    "BUILD_TESTING OFF"
    "CMAKE_POLICY_VERSION_MINIMUM 3.10"
)

target_link_libraries(fsburdf_test PRIVATE
    doctest
    FancySafeBot::fsburdf)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(fsburdf_test)
