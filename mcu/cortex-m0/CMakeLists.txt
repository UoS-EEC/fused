#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

CMAKE_MINIMUM_REQUIRED(VERSION 3.11)

add_library(
  cm0-cpu
  decode.c
  decode.h
  exmemwb.c
  exmemwb.h
  exmemwb_arith.c
  exmemwb_logic.c
  exmemwb_mem.c
  exmemwb_misc.c
  exmemwb_branch.c
  Nvic.cpp
  Nvic.hpp
  OutputPort.hpp
  sim_support.c
  sim_support.h
  SysTick.cpp
  SysTick.hpp
  )

target_include_directories(cm0-cpu PRIVATE ${CMAKE_CURRENT_LIST_DIRECTORY})

add_library(
  CortexM0Cpu
  CortexM0Cpu.cpp
  CortexM0Cpu.hpp
  )
target_link_libraries(
  CortexM0Cpu
  PRIVATE
    cm0-cpu
  PUBLIC
    Microcontroller
    PowerSystem
    systemc
    Utilities
    spdlog::spdlog
  )
