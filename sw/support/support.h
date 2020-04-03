/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#if defined(CM0_ARCH)
#include "cm0-support.h"
#elif defined(MSP430_ARCH)
#include "msp430-support.h"
#else
#error Target architecture must be defined
#endif

#include <stdbool.h>
#include <stdint.h>

// Initialize target
void target_init();

// Wait between workload iterations
void wait();

// Indicate start of workload
void indicate_workload_begin();

// Indicate end of workload
void indicate_workload_end();

// Indicate test fail
void indicate_test_fail();

// End simulation
void end_experiment();
