#
# Copyright (c) 2018-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

# Note: Call docker-build as follows if your ssh-key is needed to access any
#       external git repositories:
#
#       'docker build -t local-fused-dev . --build-arg SSH_PRIVATE_KEY="$(cat ~/.ssh/id_rsa)"'

# Stage 1: Install dependencies
FROM ubuntu:bionic as systemc-build
    WORKDIR /opt/src/
    ENV HOME /opt
    RUN apt update && apt install -y \
            libboost-dev build-essential g++ wget ninja-build git gdb rsync

    # CMAKE v3.15.4
    RUN wget https://github.com/Kitware/CMake/releases/download/v3.15.4/cmake-3.15.4-Linux-x86_64.sh &&\
        chmod a+x cmake*.sh &&\
        ./cmake*.sh --skip-license --prefix=/usr/local &&\
        rm cmake*.sh

    # Set up ssh key (safe way, not persistent)
    ARG SSH_PRIVATE_KEY
    RUN mkdir /root/.ssh/
    RUN echo "${SSH_PRIVATE_KEY}" > /root/.ssh/id_rsa && chmod 600 /root/.ssh/id_rsa
    RUN touch /root/.ssh/known_hosts
    RUN ssh-keyscan ssh-keyscan git.soton.ac.uk >> /root/.ssh/known_hosts

    # Install Fused dependencies
    ADD CMakeLists.txt /opt/src/
    ADD cmake /opt/src/cmake
    RUN mkdir -p /opt/.local
    RUN mkdir build && cd build &&\
        cmake .. -GNinja -DINSTALL_DEPENDENCIES=ON &&\
        ninja

FROM ubuntu:bionic as fused-build
    WORKDIR /opt/src/
    ENV HOME /opt
    ENV MSP430_INC /opt/.local/msp430-inc
    ENV MSP430_GCC_ROOT /opt/.local/msp430-gcc
    ENV ARM_GCC_ROOT /opt/.local/arm-gcc
    RUN apt update && apt install -y \
            g++ ninja-build build-essential libboost-dev gdb git wget unzip

    # CMAKE v3.15.4
    RUN wget https://github.com/Kitware/CMake/releases/download/v3.15.4/cmake-3.15.4-Linux-x86_64.sh &&\
        chmod a+x cmake*.sh &&\
        ./cmake*.sh --skip-license --prefix=/usr/local &&\
        rm cmake*.sh

    # Dependencies
    COPY --from=systemc-build  /opt/.local /opt/.local


