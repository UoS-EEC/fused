/*
 * Copyright (c) 2018-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

/*
 * @brief Definitions/macros for memory mapping and custom hardware/simulation
 * control in Fused
 */

#ifndef __FUSED_H
#define __FUSED_H

#ifdef CM0_ARCH
#include "cm0-fused.h"
#elif defined(MSP430_ARCH)
#include "msp430-fused.h"
#elif defined(MEMIC_ARCH)
#include "memic-fused.h"
#else
#error("Target architecture not defined")
#endif

#endif
