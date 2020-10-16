/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @brief Definitions/macros for memory mapping and custom hardware/simulation
 * control in Fused
 */

#ifndef __CM0_FUSED_H
#define __CM0_FUSED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "peripheral-defines.h"

/* ------ Memories ------ */
#define ROM_START 0x08000000                   //! Start address of ROM memory
#define ROM_SIZE 0x10000                       // 64K
#define NVRAM_START 0x20000000                 // Start address of NVRAM memory
#define NVRAM_SIZE 0x4000                      // 16K
#define SRAM_START (NVRAM_START + NVRAM_SIZE)  // Start address of SRAM memory
#define SRAM_SIZE 0x2000                       // 8K

#define PERIPHERAL_START 0x40000000  //! Start address of internal peripherals
#define NVIC_START 0xE000E100        // ! NVIC start address
#define SCB_START 0xE000E010  // ! SCB (system control block) start address

/* ------ GPIO port ------ */
#define GPIO_BASE 0x40000000
#define GPIO_SIZE 0x00001000
#define GPIO_EXCEPT_ID 0

/* ------ SimpleMonitor ------ */
#define SIMPLE_MONITOR_BASE 0x40001000
#define SIMPLE_MONITOR_SIZE 0x00000010
#define SIMPLE_MONITOR *((volatile unsigned int *)SIMPLE_MONITOR_BASE)

/* ------ SPI ------ */
#define SPI1_BASE 0x40013000
#define SPI2_BASE 0x40003800
#define SPI1_EXCEPT_ID 25
#define SPI2_EXCEPT_ID 26
#define SPI1 *((volatile unsigned int *)SPI1_BASE)
#define SPI2 *((volatile unsigned int *)SPI2_BASE)
#ifdef __cplusplus
}
#endif

#endif
