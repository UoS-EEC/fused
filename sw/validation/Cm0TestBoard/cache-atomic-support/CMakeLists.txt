#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13)
set(CODESEC F)
set(DATASEC S)
set(WS 0)
set(TESTNAME cache-atomic-support)
set(SOURCES main.c)
add_executable(${TESTNAME} ${SOURCES})
include(${CMAKE_CURRENT_LIST_DIR}/../../../cmake/tail.cmake)
