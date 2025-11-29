#!/bin/bash
set -e

# Usage function
usage() {
    echo "Usage: $0 BUILD_PRESET [OPTIONS]"
    echo ""
    echo "Arguments:"
    echo "  BUILD_PRESET       CMake build preset (required)"
    echo ""
    echo "Options:"
    echo "  --test            Run tests after building"
    echo "  --memcheck        Run tests with Valgrind memcheck"
    echo "  -h, --help        Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 debug"
    echo "  $0 debug --test"
    echo "  $0 debug --memcheck"
    exit 1
}

# Check for mandatory BUILD_PRESET
if [ $# -eq 0 ] || [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    usage
fi

BUILD_PRESET="$1"
shift

RUN_TESTS=false
RUN_MEMCHECK=false

# Parse optional arguments
while [ $# -gt 0 ]; do
    case "$1" in
        --test)
            RUN_TESTS=true
            shift
            ;;
        --memcheck)
            RUN_MEMCHECK=true
            shift
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Error: Unknown option '$1'"
            usage
            ;;
    esac
done

# Build
cmake --preset ${BUILD_PRESET}
cmake --build --preset ${BUILD_PRESET}

# Run tests if requested
if [ "$RUN_MEMCHECK" = true ]; then
    echo "Running tests with Valgrind memcheck..."
    ctest --test-dir ./build/${BUILD_PRESET} -T memcheck
elif [ "$RUN_TESTS" = true ]; then
    echo "Running tests..."
    ctest --test-dir ./build/${BUILD_PRESET}
fi
