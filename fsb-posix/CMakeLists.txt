
# headers and source files
set(FSBPOSIX_HEADERS
    include/fsb_timing.h)
set(FSBPOSIX_SOURCES
    src/fsb_timing.cpp)

# target name
set(FSBPOSIX_TARGET_NAME fsbposix)

# Generate the library from the sources
add_library(${FSBPOSIX_TARGET_NAME} STATIC
    ${FSBPOSIX_HEADERS}
    ${FSBPOSIX_SOURCES})

# library version
add_library(FancySafeBot::${FSBPOSIX_TARGET_NAME} ALIAS ${FSBPOSIX_TARGET_NAME})
set_target_properties(
    ${FSBPOSIX_TARGET_NAME}
    PROPERTIES VERSION ${PROJECT_VERSION})

# Link-time optimization
if (LTO_SUPPORTED)
    set_target_properties(${FSBPOSIX_TARGET_NAME}
        PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif ()

# include directories
target_include_directories(
    ${FSBPOSIX_TARGET_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Dependency: POSIX realtime with clock_nanosleep
include(CheckLibraryExists)
include(CheckFunctionExists)
check_library_exists(rt clock_nanosleep "" HAVE_CLOCK_NANOSLEEP)
if(HAVE_CLOCK_NANOSLEEP)
    target_link_libraries(
        ${FSBPOSIX_TARGET_NAME} PUBLIC
        rt)
else()
   set(CMAKE_EXTRA_INCLUDE_FILES time.h)
   check_function_exists(clock_nanosleep HAVE_CLOCK_NANOSLEEP)
   set(CMAKE_EXTRA_INCLUDE_FILES)
endif()
if (NOT HAVE_CLOCK_NANOSLEEP)
    message(STATUS "Could not find function clock_nanosleep(), library will use nanosleep() instead")
    target_compile_definitions(${FSBPOSIX_TARGET_NAME} PUBLIC FSB_NO_CLOCK_NANOSLEEP)
endif()

# Dependencies
target_link_libraries(
    ${FSBPOSIX_TARGET_NAME}
    PUBLIC
    FancySafeBot::fsbcore)

# Set sources for documentation (parent scope)
set(DOCUMENTATION_INPUT_${FSBPOSIX_TARGET_NAME}
    ${FSBPOSIX_HEADERS}
    ${FSBPOSIX_SOURCES}
    PARENT_SCOPE)

# test
if (FSB_BUILD_TESTING)
    include(CTest)
    add_subdirectory(test)
endif ()
