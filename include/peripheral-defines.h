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
#define SIMPLE_MONITOR_TEST_FAIL 0xFA11  //! Indicate test fail (kills sim)
#define SIMPLE_MONITOR_START_EVENT_LOG 0x000E  //! Start logging events
#define SIMPLE_MONITOR_INDICATE_BEGIN 0x0001   //! Indicate start of workload
#define SIMPLE_MONITOR_INDICATE_END 0x0002     //! Indicate end of workload

/* ------ SPI ------ */
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
