#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.12)

option(INSTALL_DEPENDENCIES "Download, build & install dependencies only" OFF)
option(ENABLE_TESTS "Build tests" ON)
option(GDB_SERVER "Link gdb server library" ON)
option(INSTALL_TARGET_TOOLCHAINS "Download & install target toolchains" ON)

set(EP_INSTALL_DIR $ENV{HOME}/.local CACHE STRING
                "Installation directory for dependencies")

IF(INSTALL_DEPENDENCIES)
  project(INSTALL_DEPENDENCIES NONE)
  include(cmake/InstallDependencies.cmake)
  return()
ELSE()
  project(fused)
ENDIF()

IF(NOT DEFINED TARGET_ARCH)
  message(FATAL_ERROR
    "TARGET_ARCH not defined, must be one of {msp430, cm0}")
ENDIF()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#Enable  sanitizer on debug build
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_RELEASE}
#       -march=native -fsanitize=address,undefined,leak")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS}
  -Wall -Wextra -Wno-reorder -Wno-unused -Wno-sign-compare")

set(CMAKE_CXX_STANDARD 11) #${SystemC_CXX_STANDARD})
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_EXE_LINKER_FLAGS -static)

IF (${TARGET_ARCH} STREQUAL "cm0")
add_compile_definitions(
  PUBLIC CM0_ARCH
  PUBLIC TARGET_LITTLE_ENDIAN
  PUBLIC TARGET_WORD_SIZE=4
  PUBLIC REG_PC=15
  PUBLIC REG_LR=14
  PUBLIC REG_SP=13
  PUBLIC REG_CPSR=0x19
  PUBLIC NUM_REGS=16
  )
ELSEIF(${TARGET_ARCH} STREQUAL "msp430")
add_compile_definitions(
  PUBLIC MSP430_ARCH
  PUBLIC TARGET_LITTLE_ENDIAN
  PUBLIC TARGET_WORD_SIZE=2
  PUBLIC REG_PC=0
  PUBLIC REG_SP=1
  PUBLIC REG_SR=2
  PUBLIC NUM_REGS=16
  )
ELSE()
  message(FATAL_ERROR "Invalid TARGET_ARCH: ${TARGET_ARCH}")
ENDIF ()

LIST(APPEND CMAKE_PREFIX_PATH ${EP_INSTALL_DIR})
find_package(spdlog REQUIRED )
find_package(ihex-parser REQUIRED)
find_package(gdb-server REQUIRED)
find_package(yaml-cpp REQUIRED)

link_directories(${EP_INSTALL_DIR}/lib)
include_directories(${EP_INSTALL_DIR}/include)

include_directories(.)

add_subdirectory(mcu)
add_subdirectory(ps)
add_subdirectory(utilities)
add_subdirectory(sd)

configure_file(config/config.yaml.in config.yaml)

add_executable(fused main.cpp)

target_link_libraries(
  fused
  Microcontroller
  PowerSystem
  SerialDevices
  systemc-ams
  systemc
  spdlog::spdlog
  Utilities
  IntelHexParser::ihex-parser
  )

IF(GDB_SERVER)
  target_link_libraries(fused gdb-server::gdb-server)
  add_compile_definitions(PUBLIC GDB_SERVER)
ENDIF()

IF(ENABLE_TESTS)
  enable_testing()
  add_subdirectory(test)
  add_test(NAME ClockSourceChannel COMMAND testClockSourceChannel)
  add_test(NAME RegisterFile COMMAND testRegisterFile)
  add_test(NAME Bme280 COMMAND testBme280)
  add_test(NAME Cache COMMAND testCache)

  IF(TARGET_ARCH STREQUAL "msp430")
    add_test(NAME DigitalIo COMMAND testDigitalIo)
    add_test(NAME Msp430fr5xxClockSystem COMMAND testMsp430fr5xxClockSystem)
    add_test(NAME Msp430fr5xxTimerA COMMAND testMsp430fr5xxTimerA)
    add_test(NAME Msp430fr5xxeUsciB COMMAND testMsp430fr5xxeUsciB)
    add_test(NAME Msp430fr5xxDma COMMAND testMsp430fr5xxDma)
  ELSEIF(TARGET_ARCH STREQUAL "cm0")
    add_test(NAME Cm0SysTick COMMAND testCm0SysTick)
    add_test(NAME Cm0Nvic COMMAND testCm0Nvic)
    add_test(NAME Cm0Spi COMMAND testCm0Spi)
  ENDIF()
ENDIF()

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

