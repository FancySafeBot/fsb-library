#!/usr/bin/env bash

set -e

CMAKE_VERSION=4.2.*
LLVM_VERSION=20

# Check for Ubuntu 22.04
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [ "$ID" != "ubuntu" ] || [ "$VERSION_ID" != "22.04" ]; then
        echo "Error: This script requires Ubuntu 22.04"
        echo "Detected: $ID $VERSION_ID"
        exit 1
    fi
else
    echo "Error: Cannot detect OS version"
    exit 1
fi

# Ensure running as root (superuser) because we modify system paths and install packages
if [ "$(id -u)" -ne 0 ]; then
    echo "Error: This script must be run as root (sudo)."
    echo "Please re-run with: sudo $0"
    exit 1
fi

# Packages
apt-get update
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
    wget
rm -rf /var/lib/apt/lists/*

# CMake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | \
    tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo 'deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ jammy main' | \
    tee /etc/apt/sources.list.d/kitware.list >/dev/null
apt-get update
apt-get install -y cmake=$CMAKE_VERSION
rm -rf /var/lib/apt/lists/*

# LLVM tools
wget -qO- https://apt.llvm.org/llvm.sh | bash -s -- ${LLVM_VERSION} all
