#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
  Msp430Peripherals
  Adc12.cpp
  Adc12.hpp
  ClockSystem.cpp
  ClockSystem.hpp
  device_includes/msp430fr5994.h
  eUSCI_B.cpp
  eUSCI_B.hpp
  DigitalIo.cpp
  DigitalIo.hpp
  Dma.cpp
  Dma.hpp
  Frctl_a.cpp
  Frctl_a.hpp
  InterruptArbiter.hpp
  Mpy32.cpp
  Mpy32.hpp
  PowerManagementModule.cpp
  PowerManagementModule.hpp
  TimerA.cpp
  TimerA.hpp
  )

target_link_libraries(Msp430Peripherals
  PUBLIC
    systemc
    PowerSystem
    Msp430Utilities
    spdlog::spdlog
  )

add_library(
  Msp430Cpu
  Msp430Cpu.cpp
  Msp430Cpu.hpp
  )

target_link_libraries(Msp430Cpu
  PUBLIC
    systemc
    PowerSystem
    Msp430Utilities
    spdlog::spdlog
  )

target_compile_definitions(
  Msp430Cpu
  PUBLIC
    MSP430_ARCH
    TARGET_LITTLE_ENDIAN
    TARGET_WORD_SIZE=2
    REG_PC=0
    REG_SP=1
    REG_SR=2
    NUM_REGS=16
  )
