@page developers_guide Developer's Guide

[TOC]

## Library Compilation

The FSB library uses CMake as the build system and is compatible with gcc and clang/LLVM compilers. The library is designed to be portable and can be built on various platforms, including Linux and macOS.

### Example Linux Build

With Ubuntu as an example, install prerequisites:

```sh
sudo apt install -y cmake ninja-build g++ graphviz doxygen
```

Build debug libraries, tests, examples, and documentation:

```sh
cmake --workflow --preset default
```

Run unit tests:

```sh
ctest --test-dir build/debug/fsb-core/test
ctest --test-dir build/debug/fsb-urdf/test
ctest --test-dir build/debug/fsb-posix/test
```

Run an example:

```sh
./build/debug/examples/body_tree_panda/fsb_body_tree_panda
```

Preview documentation by starting a server with python:

```sh
cd build/debug/doc/FancySafeBot/html
python3 -m http.server 9000
```

Then navigate to [http://localhost:9000](http://localhost:9000).

## IDE Integration

Using VSCode, you can open the library folder and use the [C++ Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools-extension-pack) to configure and build the project. The CMake Tools extension will automatically detect the CMakePresets.json file and provide options for building the project. Running the unit tests is supported with the [C++ TestMate Adapter](https://marketplace.visualstudio.com/items?itemName=matepek.vscode-catch2-test-adapter) extension.

## Contributing Code

The FSB library is open source and contributions are welcome. The following guidelines should be followed when contributing code to the library:

### Bugfixes and Small Changes

For small changes and bugfixes, please create a pull request with a clear description of the changes. The pull request should include a reference to the issue being fixed, if applicable. The pull request should also include a description of the changes and any relevant documentation updates.

### Planning a New Feature

For larger changes and new features, please create an issue to discuss the proposed changes before creating a pull request. The issue should include a clear description of the proposed changes, the motivation for the changes, and any relevant documentation updates. Once the issue has been discussed and approved, you can create a pull request with the proposed changes.

### Writing Documentation

The FSB library documentation is generated using Doxygen and is available in the `docs` folder.
All public classes and functions should be documented using Doxygen comments. The documentation should include a clear description of the class or function, the parameters, and the return value. The documentation should also include any relevant examples and usage notes.

### Writing Unit Tests

The FSB library uses Doctest for unit testing. All public classes and functions should be covered by unit tests. The unit tests should be located in the `test` subfolders and should be organized by module. The unit tests should include a clear description of the test, the expected behavior, and any relevant examples.

## Code Overview

The FSB library consists of three main components: the core library, the system calls (POSIX), and the URDF parser. The core library contains the main functionality of the library, while the system calls provides features related to multithreading, timing, and logging. A simple URD parser provides a way to load robot models from URDF files.

### Core Library

See source folder [`fsb-core`](fsb-core) for the core library.

The core library contains the main functionality of the FSB library, including the robot model tree, forward and inverse kinematics, and Jacobian calculations. The core library is designed to be used in safety-critical applications and is compliant with the MISRA C++ 2023 coding standard.

### System Calls (POSIX)

See source folder [`fsb-posix`](fsb-posix) for features requiring system calls.

The system calls provide features related to multithreading, timing, and logging. The system calls are designed to be used in safety-critical applications and are compliant with the MISRA C++ 2023 coding standard.

### URDF Parser

See source folder [`fsb-urdf`](fsb-urdf) for the URDF parser.

The URDF parser provides a way to load robot models from URDF files. The URDF parser is not MISRA compliant and should not be used in the safety-critical portion of the application. The URDF parser is intended to be used only during initialization of the robot model tree.
