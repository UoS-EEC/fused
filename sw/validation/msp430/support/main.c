/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <support.h>
#include "fused.h"

#ifndef CACHE_WARMING_ITERATIONS
#define CACHE_WARMING_ITERATIONS 1
#endif

#ifndef REPETITIONS
#define REPETITIONS 1
#endif

__attribute__((used)) int main() {
  target_init();

  while (1) {
    for (volatile long int i = 0; i < REPETITIONS + CACHE_WARMING_ITERATIONS;
         i++) {
      initialise_benchmark();
      benchmark();
      if (i == CACHE_WARMING_ITERATIONS - 1) {
        indicate_workload_begin();
      }
    }
    indicate_workload_end();
    wait();
  }
}
