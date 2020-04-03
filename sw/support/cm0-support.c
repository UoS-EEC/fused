/*
 * Copyright (c) 2018-2020, University of Southampton.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <fused.h>
#include <stdint.h>
#include <string.h>
#include "cm0-support.h"
#include "cm0.h"

void indicate_workload_begin() {
  SIMPLE_MONITOR = SIMPLE_MONITOR_INDICATE_BEGIN;
  ;
}

void indicate_workload_end() { SIMPLE_MONITOR = SIMPLE_MONITOR_INDICATE_END; }

void indicate_test_fail() { SIMPLE_MONITOR = SIMPLE_MONITOR_TEST_FAIL; }

void end_experiment() {
  SIMPLE_MONITOR = SIMPLE_MONITOR_KILL_SIM;
  while (1)
    ;  // Just in case
}

void wait() {
  for (volatile uint32_t i = 0; i < 10000ll; i++)
    ;  // delay
}

void target_init() {
  // do nothing
}

__attribute__((optimize(1), naked, used, section(".ftext"))) void _start() {
  // Boot data
  extern uint8_t __data_low, __data_high, __data_loadLow;
  if ((&__data_loadLow != &__data_low) && (&__data_low < &__data_high)) {
    memcpy(&__data_low, &__data_loadLow, &__data_high - &__data_low + 1);
  }

  int main();
  main();
}
