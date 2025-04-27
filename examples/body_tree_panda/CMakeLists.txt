
set(EXAMPLE_TARGET_NAME fsb_body_tree_panda)
set(EXAMPLE_TARGET_DESCRIPTION "Example of building a body tree and running FK using the panda robot model")

add_executable(${EXAMPLE_TARGET_NAME}
    body_tree_panda.cpp)
target_link_libraries(${EXAMPLE_TARGET_NAME} PRIVATE
    FancySafeBot::fsburdf)

# Install
install(TARGETS ${EXAMPLE_TARGET_NAME} RUNTIME DESTINATION bin)
include(CPack)

# Add to the list of examples
set(FSB_EXAMPLES ${FSB_EXAMPLES} ${EXAMPLE_TARGET_NAME} PARENT_SCOPE)
