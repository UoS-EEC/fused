/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @brief Definitions/macros of registers and commands for peripherals in fused
 */

#ifndef __PERIPHERAL_DEFINES
#define __PERIPHERAL_DEFINES

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>

/* ------ GPIO port ------ */
#define OFS_GPIO_DATA 0x0
#define OFS_GPIO_DIR 0x400
#define OFS_GPIO_IFG 0x404
#define OFS_GPIO_IE 0x410

/* ------ SimpleMonitor ------ */
#define SIMPLE_MONITOR_KILL_SIM 0x0D1E  //! Kill simulation (success)
#define SIMPLE_MONITOR_SW_ERROR 0x5D1E  //! Indicate SW error (kills simulation)
#define SIMPLE_MONITOR_TEST_FAIL 0xFA11 //! Indicate test fail (kills sim)
#define SIMPLE_MONITOR_START_EVENT_LOG 0x000E //! Start logging events
#define SIMPLE_MONITOR_INDICATE_BEGIN 0x0001  //! Indicate start of workload
#define SIMPLE_MONITOR_INDICATE_END 0x0002    //! Indicate end of workload

/* ------ SPI ------ */
#define OFS_SPI_CR1 0x00
#define OFS_SPI_CR2 0x04
#define OFS_SPI_SR 0x08
#define OFS_SPI_DR 0x0c
#define OFS_SPI_CRCPR 0x10
#define OFS_SPI_RXCRCR 0x14
#define OFS_SPI_TXCRCR 0x18

/* ------ Cache controller ------ */
#define OFS_DCACHE_CTRL_CSR 0       //! Current status register
#define OFS_DCACHE_CTRL_CRNTDIRTY 4 //! Current number of dirty lines
#define OFS_DCACHE_CTRL_MAXDIRTY 8  //! Maximum number of dirty lines
#define DCACHE_CTRL_FLUSH (1u << 0) //! Flush command)
#define DCACHE_CTRL_FASE (1u << 1)  //! Indicate Failure-Atomic Section

#define OFS_ICACHE_CTRL_CSR 0       //! Current status register
#define OFS_ICACHE_CTRL_CRNTDIRTY 4 //! Current number of dirty lines
#define OFS_ICACHE_CTRL_MAXDIRTY 8  //! Maximum number of dirty lines
#define ICACHE_CTRL_FLUSH (1u << 0) //! Flush command

/* ------ Undo logger ------ */
// Register offsets
#define OFS_UNDO_LOGGER_CTRL 0x0
#define OFS_UNDO_LOGGER_STATUS 0x4
#define OFS_UNDO_LOGGER_FIFO 0x8
#define OFS_UNDO_LOGGER_FIFO_THR 0xc
#define OFS_UNDO_LOGGER_UNSAFE_BASE 0x10
#define OFS_UNDO_LOGGER_UNSAFE_SIZE 0x14

// Control register
#define UNDO_LOGGER_CTRL_ENABLE (1u << 0)
#define UNDO_LOGGER_CTRL_CLEAR (1u << 1)
#define UNDO_LOGGER_CTRL_APPLY (1u << 2)
#define UNDO_LOGGER_CTRL_IE (1u << 3)
#define UNDO_LOGGER_CTRL_DMAEN (1u << 4)
#define UNDO_LOGGER_CTRL_FLUSHEN (1u << 5)

// Status register
#define UNDO_LOGGER_STATUS_EMPTY_SHIFT 0
#define UNDO_LOGGER_STATUS_EMPTY_MASK (1u << 0)
#define UNDO_LOGGER_STATUS_OVERFLOW_SHIFT 1
#define UNDO_LOGGER_STATUS_OVERFLOW_MASK (1u << 1)
#define UNDO_LOGGER_STATUS_THRESHOLD_SHIFT 2
#define UNDO_LOGGER_STATUS_THRESHOLD_MASK (1u << 2)
#define UNDO_LOGGER_STATUS_FREESLOTS_SHIFT 3
#define UNDO_LOGGER_STATUS_FREESLOTS_MASK (0b1111111u << 3)
#define UNDO_LOGGER_STATUS_CAPACITY_SHIFT 11
#define UNDO_LOGGER_STATUS_CAPACITY_MASK (0b1111111u << 11)

/* ------ Power/reset controller ------ */
#define POWERCONTROLLER_WARDETECTOR 0
#define POWERCONTROLLER_GPIO0 1
#define POWERCONTROLLER_GPIO1 2
#define POWERCONTROLLER_MON 3
#define POWERCONTROLLER_NVIC 4
#define POWERCONTROLLER_SCB 5
#define POWERCONTROLLER_SPI 6
#define POWERCONTROLLER_ISRAM 7
#define POWERCONTROLLER_DSRAM 8
#define POWERCONTROLLER_SYSTICK 9

/* ------ Write Tracker ------ */
#define WRITE_TRACKER_ENABLE (1u << 0)
#define WRITE_TRACKER_CLEAR (1u << 1)

#ifdef __cplusplus
}
#endif
#endif
