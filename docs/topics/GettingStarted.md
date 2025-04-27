@page getting_started Getting Started

[TOC]

Before you start using the Fancy Safe Bot (FSB) library, it's important to understand its purpose and how to integrate it into your project. The library requires you to specify the sizes of various components at compile time. It's therefore not not intended to be installed as a third party pre-compiled dependency, but rather as part of the source files for a specific robot.

## Linking the Library From Source

It's recommended to use [CPM.cmake](https://github.com/cpm-cmake/CPM.cmake) to provide the library as though you called "add_subdirectory" in CMake. This is easiest when you only want to configure the library (see below) or use a specific version but don't want to modify the source code. If you want to modify the source code, see the [Developer's Guide](/developers_guide.html) for more information.

@note
If you want to modify FSB source code, an option would be to fork on Github then have CPM use the forked repo so that contributions can still be made with Github's workflow back into the FSB project.

In your CMake file, here is an example usage with CPM:

```cmake
include (CPM.make)
CPMAddPackage(
    NAME fsb
    GITHUB_REPOSITORY FancySafeBot/fsb
    VERSION latest
    OPTIONS
    "FSB_CONFIG myrobot"
    "FSB_SIZE_BODIES 8"
    "FSB_SIZE_JOINTS 7"
    "FSB_SIZE_COORDINATES 7"
    "FSB_SIZE_DOFS 7"
)
target_link_libraries(myrobotapp PRIVATE
    FancySafeBot::fsb
    FancySafeBot::fsb-urdf)
```

Configuration options set with prefix `FSB_` are explained in the following section.

## Configuring the Library

The core FSB library does not allocate any dynamic memory so you'll need to specify sizes for your robot application at compile time. The following table provides a description of the configuration parameters that need to be set and default values used for testing the library.

@note
For developers modifying the library: Default values are required for running FSB unit tests.

| Parameter            | Default   | Description                  |
|----------------------|-----------|------------------------------|
| FSB_CONFIG           | "default" | Name of configuration.       |
| FSB_SIZE_BODIES      | 11        | Number of bodies in kinematic tree describing the robot. Must be greater than 0 |
| FSB_SIZE_JOINTS      | 10        | Number of joints in kinematic tree. Must be greater than (N - 1) for N bodies |
| FSB_SIZE_COORDINATES | 15        | Total number of generalized position coordinates for all joints in kinematic tree. |
| FSB_SIZE_DOFS        | 12        | Total number of degrees of freedom for all joints in kinematic tree. |

## Using The FSB Library

Example usage of the library can be found in the [Example Projects](example_projects.html) section. You can also follow the [Tutorials](/tutorials.html) to learn how to use the library in a step-by-step manner.
