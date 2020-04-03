#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13)

add_library(
  Microcontroller
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
  VolatileMemory.hpp
  Microcontroller.hpp
  )

IF(${TARGET_ARCH} STREQUAL "cm0")
  add_subdirectory(cortex-m0)
  target_sources(Microcontroller
    PRIVATE Cm0Microcontroller.cpp Cm0Microcontroller.hpp
    )

  target_link_libraries(Microcontroller
    CortexM0Cpu
    PowerSystem
    )
ELSEIF(${TARGET_ARCH} STREQUAL "msp430")
  add_subdirectory(msp430fr5xx)
  target_sources(Microcontroller
    PRIVATE Msp430Microcontroller.cpp Msp430Microcontroller.hpp
    )
  target_link_libraries(Microcontroller
    Msp430Cpu
    Msp430fr5xx
    PowerSystem
    )
ENDIF()
