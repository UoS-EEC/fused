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
  sim_support.c
  sim_support.h
  )

target_compile_definitions(
  cm0-cpu
  PRIVATE
    TARGET_LITTLE_ENDIAN
    TARGET_WORD_SIZE=4
  )

target_include_directories(cm0-cpu PRIVATE ${CMAKE_CURRENT_LIST_DIRECTORY})

add_library(Cm0Peripherals
  Gpio.cpp
  Gpio.hpp
  Spi.cpp
  Spi.hpp
  )

target_compile_definitions(
  Cm0Peripherals
  PRIVATE CM0_ARCH
  )

target_link_libraries(
  Cm0Peripherals
  PRIVATE
    PowerSystem
    systemc
    Cm0Utilities
    spdlog::spdlog
  )

add_library(
  CortexM0Cpu
  CortexM0Cpu.cpp
  CortexM0Cpu.hpp
  Nvic.cpp
  Nvic.hpp
  SysTick.cpp
  SysTick.hpp
  )

target_link_libraries(
  CortexM0Cpu
  PRIVATE
    cm0-cpu
  PUBLIC
    PowerSystem
    systemc
    Cm0Utilities
    spdlog::spdlog
  )

target_compile_definitions(
  CortexM0Cpu
  PUBLIC CM0_ARCH
  PUBLIC TARGET_LITTLE_ENDIAN
  PUBLIC TARGET_WORD_SIZE=4
  PUBLIC REG_PC=15
  PUBLIC REG_LR=14
  PUBLIC REG_SP=13
  PUBLIC REG_CPSR=0x19
  PUBLIC NUM_REGS=16
  )


