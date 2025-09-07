cmake_minimum_required(VERSION 3.21)

# Project #
# ####### #

# Set the project name and language
project(
    FancySafeBot
    VERSION 0.0.1
    DESCRIPTION "A gentle but powerful robotics library for embedded safety-critical applications"
    HOMEPAGE_URL "https://github.com/FancySafeBot/fsb-library"
    LANGUAGES CXX C)

set(PROJECT_BRIEF "A safe robotics library")

# Build Type #
# ########## #

# Set the possible values of build type
set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Release" "Debug" "Coverage" "Instrument" "Profile")

# Configure Options #
# ################# #

set(CMINPACK_VERSION 1.3.11 CACHE STRING "CMinpack version")
set(OPENBLAS_VERSION 0.3.30 CACHE STRING "OpenBLAS version")
set(TINYXML2_VERSION 10.0.0 CACHE STRING "TinyXML2 version")
set(DOCTEST_VERSION 2.4.11 CACHE STRING "doctest version")
set(YAMLCPP_VERSION 0.8.0 CACHE STRING "yaml-cpp version")

set(FSB_CONFIG "default" CACHE STRING "FSB Library configuration")

option(FSB_USE_OPENBLAS "Use OpenBLAS for linear algebra operations" ON)
option(FSB_USE_CMINPACK "Use CMinpack for Inverse Kinematics" OFF)
option(FSB_BUILD_TESTING "Enable Tests" ON)
option(FSB_ENABLE_DOCS "Enable doxygen documentation" OFF)
option(FSB_ENABLE_EXAMPLES "Enable building example applications" OFF)
option(FSB_EXTRA_WARNING_FLAGS "Enable extra warning options" OFF)

if (CMAKE_BUILD_TYPE STREQUAL "Release")
    option(FSB_ENABLE_LINK_TIME_OPTIMIZATION "Enables link time optimization, if available" ON)
endif ()

if (CMAKE_BUILD_TYPE STREQUAL "Coverage")
    option(FSB_ENABLE_LLVM_COVERAGE "Enables coverage build for Clang and LLVM tools" OFF)
endif()

if (CMAKE_BUILD_TYPE STREQUAL "Instrument")
    option(FSB_ENABLE_ASAN "Enable Address, Undefined Behaviour, and Memory Sanitizers" ON)
    option(FSB_ENABLE_TSAN "Enable Thread Sanitizer" OFF)
    if (FSB_ENABLE_ASAN AND FSB_ENABLE_TSAN)
        message(FATAL_ERROR "Only FSB_ENABLE_ASAN or FSB_ENABLE_TSAN can be set on ON, not both")
    endif ()
endif ()

# Fortran compiler for BLAS/LAPACK
if (FSB_USE_OPENBLAS)
    enable_language(Fortran)
endif ()

# C/C++ standard #
# ############## #

if (NOT DEFINED CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (NOT DEFINED CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Check Link-Time Optimization #
# ############################ #

if (FSB_ENABLE_LINK_TIME_OPTIMIZATION)
    include(CheckIPOSupported)
    check_ipo_supported(RESULT
        LTO_SUPPORTED
        OUTPUT
        error)
    if (LTO_SUPPORTED)
        message(STATUS "IPO / LTO enabled")
    else ()
        message(STATUS "IPO / LTO not supported: <${error}>")
    endif ()
endif ()

# CPM for dependencies
# #################### #

# see https://github.com/cpm-cmake/CPM.cmake for more info
include(cmake/CPM.cmake)

# Flags #
# ##### #

set(FSB_COMPILE_FLAGS_COMMON
    -pedantic
    -fstrict-aliasing
    -Wall
    -Wextra
    -Wformat=2
    -Wconversion
    -Wimplicit-fallthrough
    -Wformat-security
    -Wwrite-strings
    -Winit-self
    -Wcast-qual
    -Wcast-align
    -Wsign-compare
    -Wmissing-declarations
    -Wparentheses
    -Wnull-dereference
    -Wmisleading-indentation
    -Wdouble-promotion
    -Wswitch-enum
    -Wundef)

# set flags specific to build
set(FSB_LINK_FLAGS)
set(SANITIZER_FLAGS)
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    list(APPEND FSB_COMPILE_FLAGS_COMMON
        -O3
        -fno-math-errno
        -fPIE
        -fstack-protector)
    list(APPEND FSB_LINK_FLAGS
        -fPIE)
    list(APPEND SANITIZER_FLAGS
        safe-stack)
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND FSB_COMPILE_FLAGS_COMMON
        -g
        -fno-omit-frame-pointer
        -mno-omit-leaf-frame-pointer
        -funwind-tables
        -fno-optimize-sibling-calls)
elseif (CMAKE_BUILD_TYPE STREQUAL "Coverage")
    list(APPEND FSB_COMPILE_FLAGS_COMMON
        -g
        -O0
        -fno-omit-frame-pointer
        -funwind-tables
        -fprofile-arcs
        -ftest-coverage
        --coverage)
    list(APPEND FSB_LINK_FLAGS
        -fprofile-arcs
        -ftest-coverage
        --coverage)
elseif (CMAKE_BUILD_TYPE STREQUAL "Instrument")
    if (FSB_ENABLE_ASAN)
        list(APPEND FSB_COMPILE_FLAGS_COMMON
            -g
            -O1
            -fno-omit-frame-pointer
            -fno-optimize-sibling-calls
            -fsanitize=address
            -fsanitize=undefined
            -fsanitize=memory)
        list(APPEND FSB_LINK_FLAGS
            -fsanitize=address
            -fsanitize=undefined
            -fsanitize=memory)
    elseif (FSB_ENABLE_TSAN)
        list(APPEND FSB_COMPILE_FLAGS_COMMON
            -g
            -O1
            -fno-omit-frame-pointer
            -fno-optimize-sibling-calls
            -fsanitize=thread)
        list(APPEND FSB_LINK_FLAGS
            -fsanitize=thread)
    endif ()
elseif (CMAKE_BUILD_TYPE STREQUAL "Profile")
    list(APPEND COMPILE_FLAGS
        -O2
        -pg)
    list(APPEND FSB_LINK_FLAGS
        -pg)
endif ()

set(FSB_COMPILE_FLAGS_CXX
    ${FSB_COMPILE_FLAGS_COMMON}
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wsuggest-override)

set(FSB_COMPILE_FLAGS_C
    ${FSB_COMPILE_FLAGS_COMMON}
    -Wimplicit
    -Wincompatible-pointer-types
    -Wint-conversion
    -Wstrict-prototypes)

if (FSB_EXTRA_WARNING_FLAGS)
    list(APPEND FSB_COMPILE_FLAGS_COMMON
        -Wshadow
        -Wsign-conversion
        -Wswitch-default)
endif()

# print
string(REPLACE ";" "\n    " COMPILE_FLAGS_DISPLAY "${FSB_COMPILE_FLAGS_C}")
message(STATUS "C Compiler Flags:\n    ${COMPILE_FLAGS_DISPLAY}")
string(REPLACE ";" "\n    " COMPILE_FLAGS_DISPLAY "${FSB_COMPILE_FLAGS_CXX}")
message(STATUS "C++ Compiler Flags:\n    ${COMPILE_FLAGS_DISPLAY}")
string(REPLACE ";" "\n    " LINK_FLAGS_DISPLAY "${FSB_LINK_FLAGS}")
message(STATUS "Link Flags:\n    ${LINK_FLAGS_DISPLAY}")

# Subdirectories #
# ############## #

# Main library
set(FSBCORE_SUBDIR fsb-core)
add_subdirectory(${FSBCORE_SUBDIR})

# POSIX system (ITC, timers, logging, etc.)
set(FSBPOSIX_SUBDIR fsb-posix)
add_subdirectory(${FSBPOSIX_SUBDIR})

# URDF Parsing
set(FSBURDF_SUBDIR fsb-urdf)
add_subdirectory(${FSBURDF_SUBDIR})

# Documentation #
# ############# #
if (FSB_ENABLE_DOCS)
    include(cmake/GenerateDocumentation.cmake)

    # Library input source files
    list(TRANSFORM DOCUMENTATION_INPUT_fsbcore PREPEND ${FSBCORE_SUBDIR}/)
    list(TRANSFORM DOCUMENTATION_INPUT_fsbposix PREPEND ${FSBPOSIX_SUBDIR}/)
    list(TRANSFORM DOCUMENTATION_INPUT_fsburdf PREPEND ${FSBURDF_SUBDIR}/)
    set(DOCUMENTATION_INPUT_LIST
        ${DOCUMENTATION_INPUT_fsbcore}
        ${DOCUMENTATION_INPUT_fsbposix}
        ${DOCUMENTATION_INPUT_fsburdf})

    generate_documentation(
        ${PROJECT_NAME}
        ${PROJECT_VERSION}
        "${PROJECT_BRIEF}"
        "${DOCUMENTATION_INPUT_LIST}"
    )
endif ()

# Examples #
# ######## #

if (FSB_ENABLE_EXAMPLES)
    add_subdirectory(examples)
endif ()
