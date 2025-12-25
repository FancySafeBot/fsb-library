FROM ubuntu:24.04 AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV LLVM_VERSION=20

COPY ./scripts/install-ubuntu-dev-tools.sh ./install-ubuntu-dev-tools.sh
COPY ./scripts/llvm-update-alternatives.sh ./llvm-update-alternatives.sh
COPY ./scripts/install-dependencies.sh ./install-dependencies.sh

RUN ./install-ubuntu-dev-tools.sh
RUN ./llvm-update-alternatives.sh ${LLVM_VERSION}
RUN ./install-dependencies.sh

FROM base AS dev

ENV USERNAME="dev"

RUN apt-get update && apt-get install -y sudo && \
    useradd -ms /bin/bash ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER ${USERNAME}
WORKDIR /home/${USERNAME}/workspace
