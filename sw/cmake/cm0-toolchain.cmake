#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

INCLUDE(CMakeForceCompiler)

# Find toolchain programs
set(BINPATHS $ENV{ARM_GCC_ROOT}/bin)
set(BIN_PREFIX arm-none-eabi)

message(STATUS "finding ${TARGET_ARCH} compiler")
find_program(TC-GCC ${BIN_PREFIX}-gcc PATHS ${BINPATHS})
find_program(TC-GXX ${BIN_PREFIX}-g++ PATHS ${BINPATHS})
find_program(TC-OBJCOPY ${BIN_PREFIX}-objcopy PATHS ${BINPATHS})
find_program(TC-SIZE ${BIN_PREFIX}-size PATHS ${BINPATHS})
find_program(TC-OBJDUMP ${BIN_PREFIX}-objdump PATHS ${BINPATHS})

if (${TC-GCC} STREQUAL "TC-GCC-NOTFOUND")
  message(FATAL_ERROR "${BIN_PREFIX}-gcc not found in ${BINPATHS}, have you set" 
            " the environment variable ARM_GCC_ROOT correctly?")
endif()

# Define toolchain
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_ASM_COMPILER ${TC-GCC} CACHE INTERNAL "")
set(CMAKE_C_COMPILER ${TC-GCC} CACHE INTERNAL "")
set(CMAKE_CXX_COMPIER ${TC-GXX} CACHE INTERNAL "")

# Prevent CMake from testing the compilers
set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

#Debug by default
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

function(add_upload EXECUTABLE)
  add_custom_target(upload_${EXECUTABLE}
    COMMAND echo "uploading ${EXECUTABLE}..."
    COMMAND echo "upload for ${TARGET_ARCH}not implemented!"
    DEPENDS ${EXECUTABLE})
endfunction(add_upload)

# Note: not yet implemented
macro(add_all_exe)
  message(WARNING "add_all_exe not implemented for cm0, will generate
 executables with default mapping")
  FOREACH (DATASEC F S)
    FOREACH (CODESEC F S)
      FOREACH (WS 0 15)
        set(TESTNAME "${NAME}-${CODESEC}${DATASEC}-WS${WS}")
        add_executable(${TESTNAME} "")
        include(${CMAKE_CURRENT_LIST_DIR}/tail.cmake)
        FOREACH(DEF ${TARGET_DEFS})
          target_compile_definitions(${TESTNAME} PRIVATE -D${DEF})
        ENDFOREACH()
      ENDFOREACH()
    ENDFOREACH()
  ENDFOREACH()
endmacro()

macro(add_single_exe)
  set(TESTNAME "${NAME}-FS")
  set(CODESEC F)
  set(DATASEC S)
  add_executable(${TESTNAME} "")
  include(${CMAKE_CURRENT_LIST_DIR}/tail.cmake)
  FOREACH(DEF ${TARGET_DEFS})
    target_compile_definitions(${TESTNAME} PRIVATE -D${DEF})
  ENDFOREACH()
endmacro()
