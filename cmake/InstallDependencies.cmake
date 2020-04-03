#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

include (ExternalProject)

set_property (DIRECTORY PROPERTY EP_BASE Dependencies)

# SystemC
ExternalProject_Add (ep_systemc
  URL https://github.com/accellera-official/systemc/archive/2.3.3.tar.gz
  CONFIGURE_COMMAND ./configure --prefix=${EP_INSTALL_DIR}
                                CXXFLAGS=--std=c++11
                                --with-unix-layout=yes
                                --with-arch-suffix=

  BUILD_COMMAND make -j4
  BUILD_IN_SOURCE 1
  )

# SystemC-AMS
ExternalProject_Add (ep_systemc_ams
  DEPENDS ep_systemc
  URL https://www.coseda-tech.com/files/coside/user_files/Files/Proof-of-Concepts/systemc-ams-2.1.tar.gz
  CONFIGURE_COMMAND ./configure --prefix=${EP_INSTALL_DIR}
                                CXXFLAGS=--std=c++11
                                --with-systemc=${EP_INSTALL_DIR}
                                --with-arch-suffix=
  BUILD_COMMAND make -j4
  BUILD_IN_SOURCE 1
  )

# spdlog
ExternalProject_Add (ep_spdlog
  GIT_REPOSITORY https://github.com/gabime/spdlog.git
  GIT_TAG cf6f1dd01e660d5865d68bf5fa78f6376b89470a
  GIT_SHALLOW ON
  GIT_PROGRESS ON
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EP_INSTALL_DIR}
             -DSPDLOG_BUILD_TESTS=OFF
             -DSPDLOG_BUILD_EXAMPLE=OFF
  )

# yaml-cpp
ExternalProject_Add (ep_yaml_cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 9a3624205e8774953ef18f57067b3426c1c5ada6
  GIT_SHALLOW ON
  GIT_PROGRESS ON
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EP_INSTALL_DIR}
             -DYAML_CPP_BUILD_TESTS=OFF
  )

# ihex-parser
ExternalProject_Add (ep_ihex_parser
  GIT_REPOSITORY https://github.com/sivertism/ihex-parser.git
  GIT_SHALLOW ON
  GIT_PROGRESS ON
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EP_INSTALL_DIR}
  )

# GDB Server
ExternalProject_Add (ep_gdb_server
  GIT_REPOSITORY https://github.com/UoS-EEC/gdb-server.git
  GIT_SHALLOW ON
  GIT_PROGRESS ON
  CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EP_INSTALL_DIR}
  DEPENDS ep_spdlog
  )

# ------ Target compilers ------
IF (INSTALL_TARGET_TOOLCHAINS)

# MSP430-GCC -- Just download and extract
ExternalProject_Add (ep_msp430_gcc
  URL http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSPGCC/8_2_0_0/exports/msp430-gcc-8.2.0.52_linux64.tar.bz2
  CONFIGURE_COMMAND ""
  BUILD_COMMAND  ""
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND rsync -ra . ${EP_INSTALL_DIR}/msp430-gcc
  )

# MSP430 Support files -- Just download and extract
ExternalProject_Add (ep_msp430_inc
  URL http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSPGCC/8_2_0_0/exports/msp430-gcc-support-files-1.207.zip
  CONFIGURE_COMMAND ""
  BUILD_COMMAND  ""
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND rsync -ra . ${EP_INSTALL_DIR}/msp430-inc
  )

# ARM-GCC -- Just download and extract
ExternalProject_Add (ep_arm_gcc
  URL https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2
  CONFIGURE_COMMAND ""
  BUILD_COMMAND  ""
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND rsync -ra . ${EP_INSTALL_DIR}/arm-gcc
  )

ENDIF()
