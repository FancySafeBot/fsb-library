
set(FSB_EXAMPLES "")

add_subdirectory(forward_kinematics_ur5)
add_subdirectory(body_tree_panda)

add_custom_target(fsb_examples ALL DEPENDS ${FSB_EXAMPLES})
