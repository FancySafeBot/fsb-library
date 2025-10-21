FROM ubuntu:22.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

ENV CMAKE_VERSION=4.1.1
ENV LLVM_VERSION=20
ENV PLANTUML_VERSION=1.2025.8

ENV BUILD_TYPE=Release
ENV INSTALL_PREFIX=/usr/local
ENV OPT_PREFIX=/opt

RUN apt-get update && \
    apt-get -y install \
        ca-certificates \
        default-jre \
        doxygen \
        g++ \
        gcc \
        gdb \
        gfortran \
        git \
        gpg \
        graphviz \
        lsb-release \
        make \
        ninja-build \
        pkg-config \
        python3-pip \
        python3.10 \
        python3.10-distutils \
        python3.10-venv \
        rsync \
        software-properties-common \
        unzip \
        valgrind \
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

# LLVM
RUN wget -qO- https://apt.llvm.org/llvm.sh | bash -s -- ${LLVM_VERSION} all

COPY ./scripts/llvm-update-alternatives.sh ./scripts/llvm-update-alternatives.sh
RUN ./scripts/llvm-update-alternatives.sh ${LLVM_VERSION}

# PlantUML
ENV PLANTUML_URL="https://github.com/plantuml/plantuml/releases/download/v${PLANTUML_VERSION}/plantuml-${PLANTUML_VERSION}.jar"

RUN mkdir -p "/opt/plantuml" && \
    wget -O "/opt/plantuml/plantuml.jar" "$PLANTUML_URL" && \
    ln -sf "/opt/plantuml/plantuml.jar" /usr/local/bin/plantuml.jar && \
    echo "#!/bin/bash" >> /usr/local/bin/plantuml && \
    echo 'exec java -jar /usr/local/bin/plantuml.jar "$@"' >> /usr/local/bin/plantuml && \
    chmod +x /usr/local/bin/plantuml

FROM base AS build-dependencies

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

FROM build-dependencies AS build

ENV USERNAME="dev"
ENV INSTALL_PREFIX=/home/${USERNAME}/.local

RUN apt-get update && apt-get install -y sudo && \
    useradd -ms /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
    mkdir -p ${INSTALL_PREFIX}/bin

ENV PATH=$PATH:${INSTALL_PREFIX}/bin

USER ${USERNAME}
WORKDIR /home/${USERNAME}/workspace
