
set(FSB_VERSION ${FancySafeBot_VERSION})

include (cmake/ConfigureMaxSizes.cmake)
configure_max_sizes(FSBCORE_CONFIG_NAME)

# headers and source files
set(FSBCORE_HEADERS
    include/fsb_configuration.h
    include/fsb_types.h
    include/fsb_linalg3.h
    include/fsb_quaternion.h
    include/fsb_rotation.h
    include/fsb_motion.h
    include/fsb_joint.h
    include/fsb_body.h
    include/fsb_body_tree.h
    include/fsb_kinematics.h
    include/fsb_jacobian.h
    include/fsb_compute_kinematics.h
    include/fsb_trajectory_types.h
    include/fsb_trajectory_segment.h
    include/fsb_trapezoidal_velocity.h
    include/fsb_pid.h
    include/fsb_quintic.h
    include/fsb_timescale.h
    include/fsb_encoder.h
    include/fsb_spatial.h)
set(FSBCORE_SOURCES
    src/fsb_linalg3.cpp
    src/fsb_quaternion.cpp
    src/fsb_rotation.cpp
    src/fsb_motion.cpp
    src/fsb_joint.cpp
    src/fsb_body.cpp
    src/fsb_body_tree.cpp
    src/fsb_kinematics.cpp
    src/fsb_jacobian.cpp
    src/fsb_trapezoidal_velocity.cpp
    src/fsb_trajectory_segment.cpp
    src/fsb_pid.cpp
    src/fsb_quintic.cpp
    src/fsb_timescale.cpp
    src/fsb_encoder.cpp
    src/fsb_encoder.cpp
    src/fsb_spatial.cpp)

if (FSB_USE_OPENBLAS)
    list(APPEND FSBCORE_HEADERS include/fsb_linalg.h)
    list(APPEND FSBCORE_SOURCES src/fsb_linalg.c)
endif()

# target name
set(FSBCORE_TARGET_NAME fsbcore)

# Generate the library from the sources
add_library(${FSBCORE_TARGET_NAME} STATIC
    ${FSBCORE_HEADERS}
    ${FSBCORE_SOURCES})

# library version
add_library(FancySafeBot::${FSBCORE_TARGET_NAME} ALIAS ${FSBCORE_TARGET_NAME})
set_target_properties(
    ${FSBCORE_TARGET_NAME}
    PROPERTIES VERSION ${PROJECT_VERSION})

# Link-time optimization
if (LTO_SUPPORTED)
    set_target_properties(${FSBCORE_TARGET_NAME}
        PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif ()

# standard
target_compile_features(
    ${FSBCORE_TARGET_NAME} PUBLIC
    c_std_${CMAKE_C_STANDARD}
    cxx_std_${CMAKE_CXX_STANDARD})

# C/CXX flags
target_compile_options(
    ${FSBCORE_TARGET_NAME} PUBLIC
    $<$<COMPILE_LANGUAGE:CXX>:${SUPPORTED_CXX_COMPILE_FLAGS}>
    $<$<COMPILE_LANGUAGE:C>:${SUPPORTED_C_COMPILE_FLAGS}>)

# link flags
target_link_options(
    ${FSBCORE_TARGET_NAME} PUBLIC
    ${SUPPORTED_LINK_FLAGS})

# include directories
target_include_directories(
    ${FSBCORE_TARGET_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Dependency: pthread
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(
    ${FSBCORE_TARGET_NAME} PUBLIC
    Threads::Threads)

# Dependency: math extensions
check_library_exists(m exp "" HAVE_MATH_LIB)
if (HAVE_MATH_LIB)
    target_link_libraries(
        ${FSBCORE_TARGET_NAME} PUBLIC
        m)
endif ()

if (FSB_USE_OPENBLAS)
    #cmake_policy(SET CMP0077 NEW) # https://cmake.org/cmake/help/latest/policy/CMP0077.html
    #cmake_policy(SET CMP0054 NEW) # https://cmake.org/cmake/help/latest/policy/CMP0054.html
    #cmake_policy(SET CMP0048 NEW) # https://cmake.org/cmake/help/latest/policy/CMP0048.html
    set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0054 NEW)
    set(CMAKE_POLICY_DEFAULT_CMP0048 NEW)

    # Dependency: Linear Algebra OpenBLAS
    CPMAddPackage(
        NAME OpenBLAS
        GITHUB_REPOSITORY OpenMathLib/OpenBLAS
        VERSION ${OPENBLAS_VERSION}
        OPTIONS
        "BUILD_SHARED_LIBS OFF"
        "BUILD_TESTING OFF"
        "BUILD_LAPACK_DEPRECATED OFF"
        "BUILD_STATIC_LIBS ON"
        "BUILD_WITHOUT_CBLAS ON"
        "BUILD_DOUBLE ON"
        "USE_THREAD 0"
        "USE_LOCKING 1"
    )

    # Copy OpenBLAS headers (hack to put generated headers somewhere accessible without installing)
    file(COPY "${CMAKE_BINARY_DIR}/generated/f77blas.h" DESTINATION "${CMAKE_BINARY_DIR}/include/openblas")
    file(COPY "${CMAKE_BINARY_DIR}/openblas_config.h" DESTINATION "${CMAKE_BINARY_DIR}/include/openblas")
    file(COPY "${OpenBLAS_SOURCE_DIR}/lapack-netlib/LAPACKE/include/lapack.h" DESTINATION "${CMAKE_BINARY_DIR}/include/openblas")
    file(COPY "${OpenBLAS_SOURCE_DIR}/lapack-netlib/LAPACKE/include/lapacke_mangling.h" DESTINATION "${CMAKE_BINARY_DIR}/include/openblas")

    target_link_libraries(
        ${FSBCORE_TARGET_NAME} PUBLIC
        openblas_static)
endif()

if (FSB_USE_CMINPACK)
    # Dependency: CMinpack
    CPMAddPackage(
        NAME CMinpack
        GITHUB_REPOSITORY devernay/cminpack
        VERSION ${CMINPACK_VERSION}
        OPTIONS
        "CMAKE_POLICY_VERSION_MINIMUM 3.10"
        "BUILD_SHARED_LIBS OFF"
        "BUILD_TESTING OFF"
        "USE_BLAS OFF"
        "CMINPACK_PRECISION d"
    )

    target_link_libraries(
        ${FSBCORE_TARGET_NAME} PUBLIC
        cminpack)
endif()

# Set sources for documentation (parent scope)
set(DOCUMENTATION_INPUT_${FSBCORE_TARGET_NAME}
    ${FSBCORE_HEADERS}
    ${FSBCORE_SOURCES}
    PARENT_SCOPE)

# Test #
# #### #

if (FSB_BUILD_TESTING)
    if (NOT FSBCORE_CONFIG_NAME STREQUAL "default")
        message(STATUS "Unit tests are disabled with the custom configuration: ${FSBCORE_CONFIG_NAME}")
    else()
        include(CTest)
        add_subdirectory(test)
    endif()
endif ()

# Package #
# ####### #

# target_sources(${FSBCORE_TARGET_NAME} PRIVATE ${FSBCORE_SOURCES})
# target_sources(${FSBCORE_TARGET_NAME} PUBLIC
#     FILE_SET HEADERS
#     BASE_DIRS include
#     FILES ${FSBCORE_HEADERS}
# )

# include(GNUInstallDirs)
# include(CMakePackageConfigHelpers)

# configure_file(
#   ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}.pc.in
#   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
#   @ONLY
# )

# install(
#   FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc
#   DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
# )

# write_basic_package_version_file(
#   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
#   COMPATIBILITY AnyNewerVersion
# )

# install(FILES
#   ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
#   ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake
#   DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}
# )

# install(TARGETS ${FSBCORE_TARGET_NAME}
#   EXPORT ${PROJECT_NAME}
#   FILE_SET HEADERS
# )

# install(EXPORT ${PROJECT_NAME}
#     NAMESPACE  ${PROJECT_NAME}::
#     DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
