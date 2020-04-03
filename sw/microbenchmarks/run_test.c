/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <msp430fr5994.h>
#include <support.h>
#include "fram_kernels.h"
#include "sram_kernels.h"

#ifndef CODE_SECTION
#define CODE_SECTION ".text"
#endif

__attribute__((section(CODE_SECTION))) int main() {
  target_init();
  while (1) {
    cache_flush();
    indicate_workload_begin();
    RUN_TEST();  // macro set by CMAKE
    indicate_workload_end();
    wait();
  }
}
