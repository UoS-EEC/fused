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

#include "peripheral-defines.h"
#include <stdint.h>

/* ------ Memories ------ */
#define ROM_START 0x00000000 //! Start address of ROM memory
#define ROM_SIZE (128 * 1024)
#define INVM_CTRL_BASE 0x50000000

#define NVRAM_START 0x30000000 //! Start address of NVRAM memory
#define NVRAM_SIZE (32 * 1024)
#define DNVM_CTRL_BASE 0x50000010

#define ISRAM_START 0x30020000 //! Start address of ISRAM memory
#define ISRAM_SIZE (128 * 1024)
#define ISRAM_CTRL_BASE 0x50000020

#define DSRAM_START 0x30040000 //! Start address of DSRAM memory
#define DSRAM_SIZE (32 * 1024)
#define DSRAM_CTRL_BASE 0x50000030

#define PERIPHERAL_START 0x40000000 //! Start address of internal peripherals
#define NVIC_START 0xE000E100       //! NVIC start address
#define SCB_START 0xE000E010        //! SCB (system control block) start address

/* ------ GPIO port ------ */
#define GPIO0_BASE 0x40000000
#define GPIO1_BASE 0x40001000
#define GPIO_SIZE 0x00001000
#define GPIO0_EXCEPT_ID 0
#define GPIO1_EXCEPT_ID 1

/* ------ SimpleMonitor ------ */
#define SIMPLE_MONITOR_BASE 0x40002000
#define SIMPLE_MONITOR_CONSOLE_BASE 0x40002004
#define SIMPLE_MONITOR_SIZE 0x00000010
#define SIMPLE_MONITOR *((volatile unsigned int *)SIMPLE_MONITOR_BASE)
#define SIMPLE_MONITOR_CONSOLE                                                 \
  *((volatile unsigned int *)SIMPLE_MONITOR_CONSOLE_BASE)

/* ------ SPI ------ */
#define SPI1_BASE 0x40013000
#define SPI2_BASE 0x40003800
#define SPI_SIZE 20
#define SPI1_EXCEPT_ID 25
#define SPI2_EXCEPT_ID 26
#define SPI1 *((volatile unsigned int *)SPI1_BASE)
#define SPI2 *((volatile unsigned int *)SPI2_BASE)

/* ------ DMA ------ */
#define DMA_BASE 0x40014000
#define DMA_SIZE 0x74
#define DMA_EXCEPT_ID 27

/* ------ Cache controller ------ */
#define DCACHE_CTRL_BASE 0x40002020
#define DCACHE_CTRL *((unsigned int *)DCACHE_CTRL_BASE)

#define ICACHE_CTRL_BASE 0x40002040
#define ICACHE_CTRL *((unsigned int *)ICACHE_CTRL_BASE)

/* ------ Power controller ------ */
#define POWER_CONTROLLER_BASE 0x40002080
#define POWER_CONTROLLER *((volatile unsigned int *)POWER_CONTROLLER_BASE)

/* ------ Undo logger ------ */
#define UNDO_LOGGER_BASE 0x40002100
#define UNDO_LOGGER_SIZE 0x18
#define UNDO_LOGGER_EXCEPT_ID 28
#define UNDO_LOGGER_DMA_TRIGGER_CHANNEL 0
#define UNDO_LOGGER *((volatile unsigned int *)UNDO_LOGGER_BASE)

/* ------ Write tracker ------ */
#define WRITE_TRACKER_BASE 0x40003000
#define WRITE_TRACKER_BLOCK_SIZE 32
// 1 control register, plus 1 register per 32 blocks
#define WRITE_TRACKER_SIZE                                                     \
  (4 * (1 + (DSRAM_SIZE / WRITE_TRACKER_BLOCK_SIZE) / 32))

#ifdef __cplusplus
}
#endif

#endif
