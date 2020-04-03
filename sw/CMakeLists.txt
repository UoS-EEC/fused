#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.10)

IF(NOT DEFINED TARGET_ARCH)
  message(FATAL_ERROR
    "TARGET_ARCH undefined, must be one of {cm0, msp430}")
ENDIF()

IF(${TARGET_ARCH} STREQUAL "cm0")
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/cm0-toolchain.cmake)
ELSEIF(${TARGET_ARCH} STREQUAL "msp430")
  set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_LIST_DIR}/cmake/msp430-toolchain.cmake)
ELSE ()
  message(FATAL_ERROR
    "Invalid TARGET_ARCH ${TARGET_ARCH}, must be one of {cm0, msp430}")
ENDIF()

project(fused-sw)

option(SIMULATION "Enable simulation control (SIMPLE_MONITOR)" ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(cmake/common.cmake)

include_directories(support)
include_directories(../include)
include_directories(.)

add_subdirectory(support)
add_subdirectory(validation)
add_subdirectory(microbenchmarks)
