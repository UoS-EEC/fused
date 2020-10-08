#! /usr/bin/env bash
#
# Copyright (c) 2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

# ------ DESCRIPTION ------
# This script builds for both msp430 and cm0 by running cmake twice, and
# cleaning up all build files in between (so that we can use different
# toolchains).


for ARCH in cm0 msp430
do
  rm -rf build/CMake* build/.ninja* build/compile_commands.json build/rules.ninja
  cmake -Bbuild -GNinja -DTARGET_ARCH=$ARCH -DCMAKE_BUILD_TYPE=Debug
  cmake --build build
done
