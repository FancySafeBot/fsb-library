@page coding_standard Coding Standard

[TOC]

A formal coding standard is not enforced, but the following guidelines are recommended to ensure code quality and maintainability. A more comprehensive coding standard with properly configured static analysis tools would be a great contribution to the library.

The Fancy Safe Bot (FSB) library coding standard follows [MISRA C++ 2023](https://www.perforce.com/resources/qac/misra-c-cpp) and [CPP Core Guidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines) with some deviations. If you don't have access to a MISRA standard document, you may reference many of the guidelines from [µOS++ coding style](https://micro-os-plus.github.io/develop/coding-style/), [Mathworks Polyspace](https://www.mathworks.com/help/bugfinder/misra-cpp-2023-rules-and-directives.html) and SonarLint performs static analysis for [MISRA C++ 2023 rules listed here](https://rules.sonarsource.com/cpp/tag/misra-c++2023/).  ClangTidy static analysis is also applied with rules specified in [`.clang-tidy`](../../.clang-format) configuration file. 

The formatting tool [ClangFormat](https://clang.llvm.org/docs/ClangFormat.html) is used to enforce a coding style based on "WebKit" defaults and overridden rules in the [`.clang-format`](../../.clang-format) configuration file.

@note
The FSB library is not intended to be used in safety-critical applications without proper validation and verification. The coding standard is a guideline to help developers write safe and maintainable code, but it does not guarantee safety or correctness in all situations.

## The Portable Operating System Interface (POSIX)

[POSIX](https://en.wikipedia.org/wiki/POSIX) is a family of [standard Application Programming Interfaces](https://standards.ieee.org/ieee/1003.1/7700/) that allow code to be portable across operating systems. In an attempt to provide portability across Real-Time Operating Systems (RTOS), the FSB library targets [POSIX-certified](https://posix.opengroup.org/) operating systems such as Linux and QNX. FSB system calls and POSIX-related routines are optional and kept separate from the core library.

## MISRA Conformance

The FSB library is designed to be compliant with the [MISRA C++ 2023](https://www.perforce.com/resources/qac/misra-c-cpp) coding standard. The following sections outline the notable deviations from the MISRA C++ 2023 standard and the rationale behind them.

### URDF Parsing

The FSB library includes a URDF parser that is not MISRA compliant. The URDF parser is intended to only be used when initializing the robot model tree and should not be used in the safety-critical portion of the application typically during real-time operation of a robot.
