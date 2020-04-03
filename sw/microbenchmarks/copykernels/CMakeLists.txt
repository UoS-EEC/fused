#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 2.8)

set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/../msp430-toolchain.cmake")
enable_language(C ASM)

project(powerprofile)

include_directories(
    $ENV{MSP_GCC_ROOT}/include
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../common
    )
link_directories(
    $ENV{MSP_GCC_ROOT}/include
    )

IF (CODE_IN_SRAM)
    set(CODESEC ".ramtext")
ELSE()
    set(CODE_IN_SRAM 0)
    set(CODESEC ".text")
ENDIF()

set(COPYKERNELS
    memcpy
    memcpy_asm
    )

FOREACH (WS 0 15)
    FOREACH(KERNEL ${COPYKERNELS})
        FOREACH(SRC ".fram_vars" ".data")
            FOREACH(DST ".fram_vars" ".data")
                set(TESTNAME "${KERNEL}-${CODESEC}${DST}${SRC}-WS${WS}")
                string(REPLACE ".data" "S" TESTNAME ${TESTNAME})
                string(REPLACE ".ramtext" "S" TESTNAME ${TESTNAME})
                string(REPLACE ".fram_vars" "F" TESTNAME ${TESTNAME})
                string(REPLACE ".text" "F" TESTNAME ${TESTNAME})
                add_msp_executable(
                    ${TESTNAME}
                    run_copy.c
                    )
                IF (${KERNEL} STREQUAL memcpy_asm)
                    set(REPETITIONS 230)
                ELSE ()
                    set(REPETITIONS 100)
                ENDIF()
                target_compile_definitions(${TESTNAME}
                    PRIVATE -DRUN_TEST=${KERNEL}
                    PRIVATE -DSOURCE_SECTION="${SRC}"
                    PRIVATE -DDEST_SECTION="${DST}"
                    PRIVATE -DFRAM_WAIT=${WS}
                    PRIVATE -DTEXT_IN_SRAM=${CODE_IN_SRAM}
                    PRIVATE -DREPETITIONS=${REPETITIONS}
                    )
                add_msp_upload(${TESTNAME})
            ENDFOREACH()
        ENDFOREACH()
    ENDFOREACH()
ENDFOREACH()
