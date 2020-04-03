#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13)

add_library(support "")

IF(${TARGET_ARCH} STREQUAL "msp430")
  target_sources(support
    PRIVATE msp430-support.c msp430-support.h)
ELSEIF(${TARGET_ARCH} STREQUAL "cm0")
  target_sources(support
    PRIVATE cm0-support.c cm0-support.h cm0-vectors.c
    PUBLIC cm0.h)
ENDIF()
