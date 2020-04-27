#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

add_library(
  Msp430fr5xx
  Adc12.cpp
  Adc12.hpp
  ClockSystem.cpp
  ClockSystem.hpp
  device_includes/msp430fr5994.h
  eUSCI_B.cpp
  eUSCI_B.hpp
  DigitalIo.cpp
  DigitalIo.hpp
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

target_link_libraries(Msp430fr5xx
  PUBLIC
    systemc
    PowerSystem
    Utilities
    spdlog::spdlog
    Microcontroller
  )

target_compile_options(
  Msp430fr5xx PRIVATE "-Wno-unknown-pragmas"
  )

add_library(
  Msp430Cpu
  Msp430Cpu.cpp
  Msp430Cpu.hpp
  )

target_link_libraries(Msp430Cpu
  PUBLIC
    Msp430fr5xx
    systemc
    PowerSystem
    Utilities
    spdlog::spdlog
  )

target_compile_options(
  Msp430Cpu PRIVATE "-Wno-unknown-pragmas"
  )
