# headers and source files
set(FSBPOSIX_TEST_HEADERS
    fsb_test_macros.h)
set(FSBPOSIX_TEST_SOURCES
    fsb_timing_test.cpp
    fsb_test_main.cpp)

add_executable(fsbposix_test
    ${FSBPOSIX_TEST_HEADERS}
    ${FSBPOSIX_TEST_SOURCES})

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

target_link_libraries(fsbposix_test PRIVATE
    doctest
    FancySafeBot::fsbposix)

include(${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake)
doctest_discover_tests(fsbposix_test)
