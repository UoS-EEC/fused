/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

// ------ Register addresses ------
// General control register
#define ACC_CTRL 0x00

// Sampling clock divider register, divides a 10KHz sampling frequency by
// val + 1, i.e. fs = 10KHz / (CTRL_FS + 1)
#define ACC_CTRL_FS 0x04

// Status register
#define ACC_STATUS 0x08

// Data register. Note that this is a virtual read-only register where
// reads are actually forwarded to the output fifo. If the output fifo is
// empty, a read from this register returns 0, and a warning is issued.
#define ACC_DATA 0x0c

// FIFO threshold register. Sets the FIFO threshold to 4*FIFO_THR, i.e. a
// value of 16, sets the FIFO threshold to 64 bytes.
// If CTRL_IE is set, an interrupt pulse is generated when FIFO is filled
// to or beyond the FIFO threshold.
#define ACC_FIFO_THR 0x10

// ------ Bit masks & patterns ------

#define ACC_W_REG (0u << 7)
#define ACC_R_REG (1u << 7)

// CTRL Masks
#define ACC_CTRL_MODE (3u << 0)

// ------ CTRL Settings ------
//! Sleep mode, for low power consumption
#define ACC_CTRL_MODE_SLEEP (0u << 0)

//! Standby mode, ready for taking measurements
#define ACC_CTRL_MODE_STANDBY (1u << 0)

//! Continuous sampling until disabled by changing to a different mode. If
//! CTRL_IE is set, an interrupt is generated when the FIFO has filled
//! to/beyond FIFO_THR*4
#define ACC_CTRL_MODE_CONTINUOUS (2u << 0)

//! Single sampling: captures a single sample frame and returns to standby
//! mode. If CTRL_IE is set, an interrupt is generated when the FIFO is
//! nonempty
#define ACC_CTRL_MODE_SINGLE (3u << 0)

//! Software reset. Clears mode and state.
#define ACC_CTRL_SW_RESET (1u << 2)

//! Enable sampling of X axis
#define ACC_CTRL_X_EN (1u << 3)

//! Enable sampling of Y axis
#define ACC_CTRL_Y_EN (1u << 4)

//! Enable sampling of Z axis
#define ACC_CTRL_Z_EN (1u << 5)

//! Mask to extract header from CTRL register
#define ACC_CTRL_HEADER_MASK (7u << 3)

//! Amount to shift control register down by to get header
#define ACC_HEADER_SHIFT 3

//! Enable interrupt output.
#define ACC_CTRL_IE (1u << 6)

// ------ STATUS bits ------
//! Indicate a busy device. Set when sleeping and when measurements are
//! ongoing.
#define ACC_STATUS_BUSY (1u << 0)

//! Set if FIFO is filled to/beyond FIFO_THR*4 bytes
#define ACC_STATUS_OVERFLOW (1u << 1)
