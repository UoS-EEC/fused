/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef DECODE_HEADER
#define DECODE_HEADER

#include "sim_support.h"

#define DECODE_CLEAR \
  0  // Clear values from previous decode operation not overwritten by this
     // decode operation
#define DECODE_SAFE \
  1  // Breaks things! Sets duplicate decode registers just in-case the execute
     // stage uses the wrong one

// Registers that hold the results of instruction decoding
// Passed to exectue, memory access, and write-back stage in decoded variable
typedef struct {
  unsigned char rD;
  unsigned char rM;
  unsigned char rN;
  u32 imm;
  u32 cond;
  u32 reg_list;
} DECODE_RESULT;

extern DECODE_RESULT decoded;

// Interface to the decode stage
// Sets the decode stage registers based upon the passed instruction
// Prints a message and exits the simulator upon decoding error
void decode(const u16 pInsn);

#endif
