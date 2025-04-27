cmake_minimum_required(VERSION 3.21)
project(fsb_tutorial
    DESCRIPTION "FancySafeBot Tutorial for forward kinematics with a UR5 Manipulator"
    LANGUAGES CXX)
# Include FetchContent module
include(FetchContent)
# Fetch FancySafeBot library
FetchContent_Declare(
    FancySafeBot
    GIT_REPOSITORY https://github.com/FancySafeBot/fsb-library.git
    GIT_TAG main
)
FetchContent_MakeAvailable(FancySafeBot)
# Add the executable
add_executable(fsb_tutorial_fk
    src/fsb_tutorial_fk.cpp
)
# Link FancySafeBot library
target_link_libraries(fsb_tutorial_fk
    PRIVATE FancySafeBot::fsburdf
)
