/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @brief Definitions/macros for memory mapping and custom hardware/simulation
 * control in Fused.
 */

#ifndef __MSP430_FUSED_H
#define __MSP430_FUSED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "peripheral-defines.h"

/* ------ Memories ------ */
#define NVRAM_START 0x4000  // Start address of NVRAM memory
#define NVRAM_SIZE 0xBF80   // ~49K
#define SRAM_START 0x1C00   // Start address of SRAM memory
#define SRAM_SIZE 0x2000    // 8K

/* ------ Custom peripherals ------ */
#define PERIPHERAL_START 0x0B00  //! Start address of internal peripherals

/* ------ SimpleMonitor ------ */
#define SIMPLE_MONITOR_BASE PERIPHERAL_START
#define SIMPLE_MONITOR_SIZE 0x0010
#define SIMPLE_MONITOR *((unsigned int *)SIMPLE_MONITOR_BASE)

#ifdef __cplusplus
}
#endif

#endif
