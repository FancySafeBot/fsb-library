#!/usr/bin/env bash

set -e

# Parse command line arguments
USE_SUDO=false
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --root|--sudo) USE_SUDO=true ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
    shift
done

BUILD_TYPE=Release
OPENBLAS_VERSION=0.3.30
BUILD_DIR=build/dependencies

this_dir=$(pwd)
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"
rm -rf OpenBLAS

# Clone
git clone -b v${OPENBLAS_VERSION} --depth 1 https://github.com/OpenMathLib/OpenBLAS.git
cd OpenBLAS

# Make and install
cmake -S . -B cmake-build \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_TESTING=OFF \
        -DBUILD_LAPACK_DEPRECATED=OFF \
        -DBUILD_STATIC_LIBS=ON \
        -DBUILD_WITHOUT_CBLAS=ON \
        -DBUILD_DOUBLE=ON \
        -DUSE_THREAD=0 \
        -DUSE_LOCKING=1
cmake --build cmake-build -j$(nproc)

# Install with or without sudo
if [ "$USE_SUDO" = true ]; then
    sudo cmake --install cmake-build
else
    cmake --install cmake-build
fi

# Cleanup
cd ..
rm -rf OpenBLAS

cd "${this_dir}"
