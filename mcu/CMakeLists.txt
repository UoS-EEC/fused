#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13)

add_subdirectory(cortex-m0)
add_subdirectory(msp430fr5xx)

set(COMMON_SOURCES
  Bus.cpp
  Bus.hpp
  BusTarget.cpp
  BusTarget.hpp
  Cache.cpp
  Cache.hpp
  CacheReplacementPolicies.hpp
  ClockDivider.hpp
  ClockMux.hpp
  ClockSourceIf.hpp
  ClockSourceChannel.hpp
  DummyPeripheral.cpp
  DummyPeripheral.hpp
  DynamicClock.cpp
  DynamicClock.hpp
  GenericMemory.cpp
  GenericMemory.hpp
  Microcontroller.cpp
  NonvolatileMemory.hpp
  NonvolatileMemory.cpp
  RegisterFile.cpp
  RegisterFile.hpp
  SpiTransactionExtension.hpp
  VolatileMemory.hpp
  Microcontroller.hpp
  )

add_library(Cm0Microcontroller)
target_sources(
  Cm0Microcontroller
  PRIVATE
    ${COMMON_SOURCES}
    Cm0Microcontroller.cpp
    Cm0Microcontroller.hpp
  )

target_link_libraries(
  Cm0Microcontroller
  PUBLIC
    CortexM0Cpu
    Cm0Peripherals
    PowerSystem
)

add_library(Msp430Microcontroller)
target_sources(
  Msp430Microcontroller
  PRIVATE
    ${COMMON_SOURCES}
    Msp430Microcontroller.cpp
    Msp430Microcontroller.hpp
  )

target_link_libraries(
  Msp430Microcontroller
  PUBLIC
    Msp430Cpu
    Msp430Peripherals
    PowerSystem
  )

