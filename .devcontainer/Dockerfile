FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

ENV CMAKE_VERSION=4.1.1
ENV CLANGD_VERSION=20.1.8

ENV BUILD_TYPE=Release
ENV INSTALL_PREFIX=/usr/local
ENV OPT_PREFIX=/opt

RUN apt-get update && \
    apt-get -y install \
        ca-certificates \
        doxygen \
        make \
        g++ \
        gcc \
        gdb \
        gfortran \
        git \
        gpg \
        graphviz \
        make \
        ninja-build \
        pkg-config \
        python3-pip \
        python3.10 \
        python3.10-distutils \
        python3.10-venv \
        rsync \
        unzip \
        wget && \
    rm -rf /var/lib/apt/lists/*

# Install CMake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | \
    tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null && \
    echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null && \
    apt-get update && \
    apt-get install -y cmake && \
    rm -rf /var/lib/apt/lists/*

FROM base AS build-libraries

ENV OPENBLAS_VERSION=0.3.30
WORKDIR /build
RUN git clone -b v${OPENBLAS_VERSION} --depth 1 https://github.com/OpenMathLib/OpenBLAS.git
WORKDIR /build/OpenBLAS

RUN cmake -S . -B cmake-build \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_PREFIX} \
        -DBUILD_SHARED_LIBS=OFF \
        -DBUILD_TESTING=OFF \
        -DBUILD_LAPACK_DEPRECATED=OFF \
        -DBUILD_STATIC_LIBS=ON \
        -DBUILD_WITHOUT_CBLAS=ON \
        -DBUILD_DOUBLE=ON \
        -DUSE_THREAD=0 \
        -DUSE_LOCKING=1 && \
    cmake --build cmake-build -j$(nproc) && \
    cmake --install cmake-build

RUN ldconfig

WORKDIR /build
RUN rm -rf OpenBLAS

FROM build-libraries AS devcontainer

ENV DEV_USERNAME="dev"
ENV DEV_INSTALL_PREFIX=/home/${DEV_USERNAME}/.local

RUN apt-get update && apt-get install -y sudo && \
    useradd -ms /bin/bash ${DEV_USERNAME} && \
    echo "${DEV_USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p ${DEV_INSTALL_PREFIX}/bin

ENV PATH=$PATH:${DEV_INSTALL_PREFIX}/bin

USER ${DEV_USERNAME}
WORKDIR /home/${DEV_USERNAME}/workspace
