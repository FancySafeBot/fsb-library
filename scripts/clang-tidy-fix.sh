#!/bin/bash
# Run clang-tidy with automatic fixes on the codebase
# Usage: ./scripts/clang-tidy-fix.sh [path]

set -e

BUILD_DIR="build/debug"
COMPILE_COMMANDS="${BUILD_DIR}/compile_commands.json"

# Check if compile_commands.json exists
if [ ! -f "${COMPILE_COMMANDS}" ]; then
    echo "Error: ${COMPILE_COMMANDS} not found"
    echo "Please build the project first with: cmake --preset debug"
    exit 1
fi

# Determine which paths to fix
if [ -n "$1" ]; then
    PATHS=("$1")
else
    PATHS=("fsb-core" "fsb-posix" "fsb-urdf")
fi

# Find all C++ source files
SOURCES=()
for path in "${PATHS[@]}"; do
    while IFS= read -r -d '' file; do
        SOURCES+=("$file")
    done < <(find "$path" -type f \( -name "*.cpp" -o -name "*.cxx" -o -name "*.cc" \) -print0 2>/dev/null)
done

if [ ${#SOURCES[@]} -eq 0 ]; then
    echo "No source files found in: ${PATHS[*]}"
    exit 1
fi

echo "Found ${#SOURCES[@]} source files"

# Run clang-tidy with fixes
for source in "${SOURCES[@]}"; do
    echo "Processing: ${source}"
    clang-tidy -p="${BUILD_DIR}" --fix --fix-errors "${source}" 2>&1 | grep -v "warnings generated" || true
done

echo "Done"
