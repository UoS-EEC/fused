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
set(TESTNAME spi)
set(SOURCES 
  main.c 
  eusci_b_spi.c 
  eusci_b_spi.h
  )
add_executable(${TESTNAME} ${SOURCES})
include(../../../cmake/tail.cmake)
