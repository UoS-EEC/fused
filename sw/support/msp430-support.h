/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <fused.h>
#include <msp430fr5994.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifndef FRAM_WAIT
#define FRAM_WAIT 0  // Number of wait states on cache miss (0-15)
#endif

// Hand-written fast version of memcpy
void fastmemcpy(uint8_t *dst, uint8_t *src, size_t len);

// Enable interrupts
void enable_interrupts();

// Disable interrupts
void disable_interrupts();

// "Flush cache" (by loading nops)
void cache_flush();
