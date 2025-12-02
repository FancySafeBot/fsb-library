#!/usr/bin/env bash

set -e

BUILD_TYPE=Release
OPENBLAS_VERSION=0.3.30

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
cmake --install cmake-build

# Cleanup
cd ..
rm -rf OpenBLAS
