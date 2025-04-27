
set(EXAMPLE_TARGET_NAME fsb_forward_kinematics_ur5)
set(EXAMPLE_TARGET_DESCRIPTION "Example of using FancySafeBot for forward kinematics with a UR5 Manipulator")

add_executable(${EXAMPLE_TARGET_NAME}
    forward_kinematics_ur5.cpp)

# Set target description
set_target_properties(${EXAMPLE_TARGET_NAME} PROPERTIES
    DESCRIPTION "${EXAMPLE_TARGET_DESCRIPTION}"
)

# dependency: argparse
CPMAddPackage(
    NAME argparse
    GITHUB_REPOSITORY p-ranav/argparse
    VERSION 3.2
    OPTIONS
    "BUILD_SHARED_LIBS OFF"
    "BUILD_TESTING OFF"
)

target_link_libraries(${EXAMPLE_TARGET_NAME} PRIVATE
    argparse::argparse
    FancySafeBot::fsburdf)

# Copy data
file(COPY data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

# Install
install(TARGETS ${EXAMPLE_TARGET_NAME} RUNTIME DESTINATION bin)
include(CPack)

# Add to the list of examples
set(FSB_EXAMPLES ${FSB_EXAMPLES} ${EXAMPLE_TARGET_NAME} PARENT_SCOPE)
