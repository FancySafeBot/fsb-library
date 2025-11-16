#!/bin/bash
# Run clang-tidy with automatic fixes on the codebase
# Usage: ./scripts/clang-tidy-fix.sh [path]

set -e

# Determine which build directory
if [ -n "$1" ]; then
    PRESET="$1"
else
    PRESET="debug"
fi

# Parse arguments
NO_FIX=false
if [ "$2" = "--no-fix" ]; then
    NO_FIX=true
fi

BUILD_DIR="build/${PRESET}"
COMPILE_COMMANDS="${BUILD_DIR}/compile_commands.json"
PATHS=("fsb-core" "fsb-posix" "fsb-urdf")

# Check if compile_commands.json exists
if [ ! -f "${COMPILE_COMMANDS}" ]; then
    echo "Error: ${COMPILE_COMMANDS} not found"
    echo "Please build the project first with: cmake --preset ${PRESET}"
    exit 1
fi

# Find all C++ source files (excluding files ending in test_main.cpp and test subfolders)
SOURCES=()
for path in "${PATHS[@]}"; do
    while IFS= read -r -d '' file; do
        if [[ ! "$file" =~ /test/ ]]; then
            SOURCES+=("$file")
        fi
    done < <(find "$path" -type f \( -name "*.cpp" -o -name "*.cxx" -o -name "*.cc" \) -print0 2>/dev/null)
done

if [ ${#SOURCES[@]} -eq 0 ]; then
    echo "No source files found in: ${PATHS[*]}"
    exit 1
fi

echo "Found ${#SOURCES[@]} source files"

# Run clang-tidy with or without fixes
if [ "$NO_FIX" = true ]; then
    # Run without fixes
    for source in "${SOURCES[@]}"; do
        echo "Processing: ${source}"
        clang-tidy -p="${BUILD_DIR}" "${source}" 2>&1 | grep -vE "(warnings generated|Suppressed .* warnings|Use -header-filter=|Use -system-headers)" || true
    done
else
    # Run with fixes
    for source in "${SOURCES[@]}"; do
        echo "Processing: ${source}"
        clang-tidy -p="${BUILD_DIR}" --fix --fix-errors "${source}" 2>&1 | grep -vE "(warnings generated|Suppressed .* warnings|Use -header-filter=|Use -system-headers)" || true
    done
fi

echo "Done"
