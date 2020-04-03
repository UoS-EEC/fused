#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 2.8)

enable_language(C ASM)

IF(${TARGET_ARCH} STREQUAL "msp430")

  set(CODE_IN_SRAM 0)

  set(KERNELS
    kernel_fram
    kernel_sram
    kernel_fram_wsingle
    kernel_fram_wsinglesparse
    kernel_cache_linear
    kernel_cache_sparse
    kernel_registers
    kernel_fram_raw
    kernel_sram_wsingle
    LRUTest
    TI0HitExample
    )

  FOREACH(WS 0 15)
      FOREACH(KERNEL ${KERNELS})
          set(TESTNAME "${KERNEL}-WS${WS}")
          message(${TESTNAME})
          string(REPLACE "kernel_" "k" TESTNAME ${TESTNAME})
          string(REPLACE "fram" "F" TESTNAME ${TESTNAME})
          string(REPLACE "sram" "S" TESTNAME ${TESTNAME})
          add_executable(
            ${TESTNAME}
            run_test.c
            fram_kernels.S
            fram_kernels.h
            sram_kernels.S
            sram_kernels.h)
          target_compile_definitions(${TESTNAME}
            PRIVATE -DRUN_TEST=${KERNEL}
            PRIVATE -DFRAM_WAIT=${WS}
            PRIVATE -DDATA_IN_SRAM=1
            )
          target_link_libraries(${TESTNAME} support ${SUPPORT_LIBS})
          target_link_options(${TESTNAME}
            PUBLIC -T ../support/msp430fr5994-FS.ld)
          add_upload(${TESTNAME})
      ENDFOREACH()
  ENDFOREACH()
ENDIF()
