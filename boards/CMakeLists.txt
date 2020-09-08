#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.13)

# ------ CortexM0 Test board ------
add_executable(
  Cm0TestBoard
  Cm0TestBoard.cpp
  )

target_link_libraries(
  Cm0TestBoard
  PUBLIC
    Cm0Microcontroller
    PowerSystem
    SerialDevices
    systemc-ams
    systemc
    spdlog::spdlog
    Cm0Utilities
    IntelHexParser::ihex-parser
  )

IF(GDB_SERVER)
  target_link_libraries(Cm0TestBoard PUBLIC gdb-server::gdb-server)
  target_compile_definitions(Cm0TestBoard PUBLIC GDB_SERVER)
ENDIF()

# ------ MSP430 Test board ------
add_executable(
  Msp430TestBoard
  Msp430TestBoard.cpp
  )

target_link_libraries(
  Msp430TestBoard
  PRIVATE
    Msp430Microcontroller
    PowerSystem
    SerialDevices
    systemc-ams
    systemc
    spdlog::spdlog
    Msp430Utilities
    IntelHexParser::ihex-parser
  )

IF(GDB_SERVER)
  target_link_libraries(Msp430TestBoard PRIVATE gdb-server::gdb-server)
  target_compile_definitions(Msp430TestBoard PUBLIC GDB_SERVER)
ENDIF()

# ------ MSP430FR5994Launchpad ------
add_executable(
  Msp430FR5994Launchpad
  Msp430FR5994Launchpad.cpp
  )

target_link_libraries(
  Msp430FR5994Launchpad
  PRIVATE
    Msp430Microcontroller
    PowerSystem
    SerialDevices
    Header
    systemc-ams
    systemc
    spdlog::spdlog
    Msp430Utilities
    IntelHexParser::ihex-parser
  )

IF(GDB_SERVER)
  target_link_libraries(Msp430FR5994Launchpad PRIVATE gdb-server::gdb-server)
  target_compile_definitions(Msp430FR5994Launchpad PUBLIC GDB_SERVER)
ENDIF()

