
# headers and source files
set(FSBURDF_HEADERS
    include/fsb_urdf_utilities.h
    include/fsb_urdf_error.h
    include/fsb_urdf_name_map.h
    include/fsb_urdf_joint.h
    include/fsb_urdf_origin.h
    include/fsb_urdf_inertial.h
    include/fsb_urdf.h)
set(FSBURDF_SOURCES
    src/fsb_urdf_utilities.cpp
    src/fsb_urdf_body.cpp
    src/fsb_urdf_origin.cpp
    src/fsb_urdf_inertial.cpp
    src/fsb_urdf_joint.cpp
    src/fsb_urdf.cpp)

# target name
set(FSBURDF_TARGET_NAME fsburdf)

# Generate the library from the sources
add_library(${FSBURDF_TARGET_NAME} STATIC
    ${FSBURDF_HEADERS}
    ${FSBURDF_SOURCES})

# library version
add_library(FancySafeBot::${FSBURDF_TARGET_NAME} ALIAS ${FSBURDF_TARGET_NAME})
set_target_properties(
    ${FSBURDF_TARGET_NAME}
    PROPERTIES VERSION ${PROJECT_VERSION})

# Link-time optimization
if (LTO_SUPPORTED)
    set_target_properties(${FSBURDF_TARGET_NAME}
        PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif ()

# include directories
target_include_directories(
    ${FSBURDF_TARGET_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# standard
target_compile_features(
    ${FSBURDF_TARGET_NAME} PUBLIC
    c_std_99
    cxx_std_20)

# Dependency: TinyXML
CPMAddPackage(
    NAME tinyxml2
    GITHUB_REPOSITORY leethomason/tinyxml2
    VERSION ${TINYXML2_VERSION}
    OPTIONS
    "BUILD_TESTING OFF"
    "BUILD_APPS OFF"
    "BUILD_SHARED_LIBS OFF"
)

# Dependencies
target_link_libraries(
    ${FSBURDF_TARGET_NAME}
    PUBLIC
    tinyxml2::tinyxml2
    FancySafeBot::fsbcore)

# Set sources for documentation (parent scope)
set(DOCUMENTATION_INPUT_${FSBURDF_TARGET_NAME}
    ${FSBURDF_HEADERS}
    ${FSBURDF_SOURCES}
    PARENT_SCOPE)

# test
if (FSB_BUILD_TESTING)
    include(CTest)
    add_subdirectory(test)
endif ()
