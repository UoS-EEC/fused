/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef CM0_ARCH
#include "cm0-support.h"
#elif defined(MSP430_ARCH)
#include "msp430-support.h"
#else
#error Target architecture must be defined
#endif

// Functions that must be implemented by benchmarks

/**
 * @brief Verify benchmark result
 * @param result return value of benchmark
 * @return 0 if success, -1 if no verification done, 1 otherwise
 */
int verify_benchmark(int result);

/**
 * @brief Initialise benchmark.
 */
void initialise_benchmark(void);

/**
 * @brief Run benchmark.
 * @return benchmark-specific return value. For example result of computation.
 */
int benchmark(void) __attribute__((noinline));
