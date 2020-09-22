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

/* ------ Memories ------ */
#define ROM_START 0x08000000                   //! Start address of ROM memory
#define ROM_SIZE 0x4000                        // 16K
#define NVRAM_START 0x20000000                 // Start address of NVRAM memory
#define NVRAM_SIZE 0x4000                      // 16K
#define SRAM_START (NVRAM_START + NVRAM_SIZE)  // Start address of SRAM memory
#define SRAM_SIZE 0x2000                       // 8K

#define PERIPHERAL_START 0x40000000  //! Start address of internal peripherals
#define NVIC_START 0xE000E100        // ! NVIC start address
#define SCB_START 0xE000E010  // ! SCB (system control block) start address

/* ------ SimpleMonitor ------ */
#define SIMPLE_MONITOR_BASE 0x40000000
#define SIMPLE_MONITOR_SIZE 0x4
#define SIMPLE_MONITOR *((volatile unsigned int *)SIMPLE_MONITOR_BASE)
#define SIMPLE_MONITOR_KILL_SIM 0x0D1E  //! Kill simulation (success)
#define SIMPLE_MONITOR_SW_ERROR 0x5D1E  //! Indicate SW error (kills simulation)
#define SIMPLE_MONITOR_TEST_FAIL 0xFA11  //! Indicate test fail (kills sim)
#define SIMPLE_MONITOR_START_EVENT_LOG 0x000E  //! Start logging events
#define SIMPLE_MONITOR_INDICATE_BEGIN 0x0001   //! Indicate start of workload
#define SIMPLE_MONITOR_INDICATE_END 0x0002     //! Indicate end of workload

/* ------ Output port ------ */
#define OUTPORT_BASE 0x40000010  //! Output port base address
#define OUTPORT *((volatile unsigned int *)OUTPORT_BASE)
#define OFS_OUTPORT_OUT 0  //! Output port register

/* ------ SPI ------ */
#define SPI1_BASE 0x40013000
#define SPI2_BASE 0x40003800
#define SPI1_EXCEPT_ID 25
#define SPI2_EXCEPT_ID 26
#define SPI1 *((volatile unsigned int *)SPI1_BASE)
#define SPI2 *((volatile unsigned int *)SPI2_BASE)
#define OFS_SPI_CR1 0x00
#define OFS_SPI_CR2 0x04
#define OFS_SPI_SR 0x08
#define OFS_SPI_DR 0x0c
#define OFS_SPI_CRCPR 0x10
#define OFS_SPI_RXCRCR 0x14
#define OFS_SPI_TXCRCR 0x18

#ifdef __cplusplus
}
#endif

#endif
