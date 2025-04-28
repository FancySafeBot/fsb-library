@page tutorial_forward_kinematics Tutorial - Forward Kinematics

[TOC]

For this tutorial we will use the `fsb::ComputeKinematics` class to compute the forward kinematics of a robot. The forward kinematics computes the pose, velocity, and acceleration of all rigid bodies of the robot given the joint positions, velocities, and accelerations. A UR5 robot is used as an example.

## Project Setup

To set up the project, create a new directory and add source files:

```sh
touch CMakeLists.txt
mkdir -p src && touch src/fsb_tutorial_fk.cpp
```

Initialize the CMakeLists.txt file with the following lines:

```cmake
cmake_minimum_required(VERSION 3.21)
project(fsb_tutorial
    DESCRIPTION "FancySafeBot Tutorial for forward kinematics with a UR5 Manipulator"
    LANGUAGES C CXX)
```

Add the fancysafebot URDF parser and core library to the CMakeLists.txt file:

```cmake
# Include FetchContent module
include(FetchContent)
# Fetch FancySafeBot library
FetchContent_Declare(
    FancySafeBot
    GIT_REPOSITORY https://github.com/FancySafeBot/fsb-library.git
    GIT_TAG main
)
FetchContent_MakeAvailable(FancySafeBot)
```

Add the executable target for the tutorial:

```cmake
# Add the executable
add_executable(fsb_tutorial_fk
    src/fsb_tutorial_fk.cpp
)
# Link FancySafeBot library
target_link_libraries(fsb_tutorial_fk
    PRIVATE FancySafeBot::fsburdf
)
```

Now we are ready to write the code! The code will be written in the `fsb_tutorial_fk.cpp` file. Initialize the file with the following lines:

```cpp
#include <iostream>
int main(void) {
    std::cout <<
        "FancySafeBot Tutorial for forward kinematics with a UR5 Manipulator\n";
    return EXIT_SUCCESS;
}
```

You can build and run the application with the following commands:

```sh
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Debug
cd build
ninja fsb_tutorial_fk
./fsb_tutorial_fk
```

## Loading the Robot Model

The robot model is loaded from a URDF file. For this example, we can copy over the URDF file from the `fsb-library` repository test data at [github.com/FancySafeBot/fsb-library/fsb-urdf/test/data/ur5/ur5.urdf](https://github.com/FancySafeBot/fsb-library/fsb-urdf/test/data/ur5/ur5.urdf) to the current directory.

```sh
mkdir -p data && wget https://github.com/FancySafeBot/fsb-library/blob/main/fsb-urdf/test/data/ur5/ur5.urdf -O data/ur5.urdf
```

```cpp
// parse URDF
const std::string urdf_path = "data/ur5.urdf";
fsb::urdf::UrdfError   urdf_err = {};
fsb::urdf::UrdfNameMap name_map = {};
const fsb::BodyTree    body_tree = fsb::urdf::parse_urdf_file(urdf_path, name_map, urdf_err);
if (urdf_err.is_error())
{
    std::cerr << "Error parsing URDF file: " << urdf_err.get_description() << "\n";
    return EXIT_FAILURE;
}
```

## Calculating Forward Kinematics

The forward kinematics can be calculated using the `fsb::ComputeKinematics` class. The class provides methods to compute the forward kinematics of the robot given the joint positions, velocities, and accelerations.

```cpp
// Initialize kinematics object
fsb::ComputeKinematics kinematics = {};
kinematics.initialize(body_tree);
```

Compute the forward kinematics of the robot using the `compute_forward_kinematics_pose` method. The method takes the joint positions as input and returns the pose of all rigid bodies of the robot.

```cpp
// Input joint input position
const fsb::JointSpacePosition joint_position = {};
// compute forward kinematics
fsb::BodyCartesianPva cartesian_pva = {};
kinematics.compute_forward_kinematics_pose(joint_position, cartesian_pva);
```

If we want to get the pose of the end effector, we can specify the end effector name to get the body index from the name map.:

```cpp
const std::string end_effector_name = "ee_link";
fsb::urdf::NameMapError name_err = {};
const size_t ee_index = name_map.get_body_index(end_effector_name, name_err);
if (name_err != fsb::urdf::NameMapError::SUCCESS)
{
    std::cerr << "Body name '" << end_effector_name << "' not found in URDF file " << urdf_path << "\n";
    return EXIT_FAILURE;
}
```

Finally, we can print the pose of the end effector:

```cpp
// Print the pose of the end effector
const fsb::Transform& body_pose = cartesian_pva.body[ee_index].pose;
std::cout << "End effector pose:\n";
std::cout << "  Position: (" << ee_pose.translation.x << ", " << ee_pose.translation.y << ", " << ee_pose.translation.z << ")\n";
std::cout << "  Orientation: (" << ee_pose.rotation.qx << ", " << ee_pose.rotation.qy << ", " << ee_pose.rotation.qz << ", " << ee_pose.rotation.qw << ")\n";
```
