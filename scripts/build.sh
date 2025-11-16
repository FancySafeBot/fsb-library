#!/bin/bash
set -e

BUILD_PRESET="$1"

cmake --preset ${BUILD_PRESET}
cmake --build --preset ${BUILD_PRESET}

if [ "$1" = "test" ]; then
    ctest --test-dir ./build/${BUILD_PRESET}/fsb-core/test
    ctest --test-dir ./build/${BUILD_PRESET}/fsb-posix/test
    ctest --test-dir ./build/${BUILD_PRESET}/fsb-urdf/test
fi
