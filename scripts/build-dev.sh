#!/bin/bash
set -e

cmake --preset dev-debug
cmake --build --preset dev-debug

if [ "$1" = "test" ]; then
    ctest --test-dir ./build/dev-debug/fsb-core/test
    ctest --test-dir ./build/dev-debug/fsb-posix/test
    ctest --test-dir ./build/dev-debug/fsb-urdf/test
fi
