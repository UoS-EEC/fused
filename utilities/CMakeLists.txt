#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 2.8)

set(SOURCES
  BoolLogicConverter.hpp
  Config.cpp
  Config.hpp
  Utilities.cpp
  Utilities.hpp
  IoSimulationStopper.hpp
  SimpleMonitor.hpp
  SimulationController.cpp
  SimulationController.hpp
  )

add_library(Cm0Utilities ${SOURCES})

target_link_libraries(
  Cm0Utilities
  PRIVATE systemc-ams
  PRIVATE systemc
  PRIVATE yaml-cpp
  )

target_compile_definitions(
  Cm0Utilities
  PRIVATE TARGET_LITTLE_ENDIAN
  PRIVATE TARGET_WORD_SIZE=4
  )

add_library(Msp430Utilities ${SOURCES})

target_link_libraries(
  Msp430Utilities
  PRIVATE systemc-ams
  PRIVATE systemc
  PRIVATE yaml-cpp
  )

target_compile_definitions(
  Msp430Utilities
  PRIVATE TARGET_LITTLE_ENDIAN
  PRIVATE TARGET_WORD_SIZE=2
  )
