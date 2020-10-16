/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <msp430fr5994.h>
#include <stdint.h>
#include <string.h>

#ifndef FRAM_WAIT
#define FRAM_WAIT 0  // Number of wait states on cache miss (0-15)
#endif

#if TEXT_IN_SRAM && DATA_IN_SRAM
#define FRAM_WAIT_MULT 4
#elif TEXT_IN_SRAM
#define FRAM_WAIT_MULT (4 - FRAM_WAIT / 8)
#elif DATA_IN_SRAM
#define FRAM_WAIT_MULT (4 - FRAM_WAIT / 8)
#else
#define FRAM_WAIT_MULT (4 - FRAM_WAIT / 8)
#endif

#define DO_WAIT

#ifdef DO_WAIT
#define WAIT                                                       \
  for (volatile uint32_t i = 0; i < FRAM_WAIT_MULT * 10000ll; i++) \
    ;  // delay
#else
#define WAIT  // Do nothing
#endif

void target_init();
void wait();
void indicate_begin();
void indicate_end();
