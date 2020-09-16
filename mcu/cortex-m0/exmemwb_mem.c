/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>
#include "decode.h"
#include "exmemwb.h"
#include "sim_support.h"

///--- Load/store multiple operations
///--------------------------------------------///

// LDM - Load multiple registers from the stack
u32 ldm() {
  u32 numLoaded = 0;
  u32 rNWritten = (1 << decoded.rN) & decoded.reg_list;
  u32 address = cpu_get_gpr(decoded.rN);

  for (int i = 0; i < 8; ++i) {
    int mask = 1 << i;
    if (decoded.reg_list & mask) {
      u32 data = 0;
      simLoadWord(address, &data);
      cpu_set_gpr(i, data);
      address += 4;
      ++numLoaded;
    }
  }

  if (rNWritten == 0) cpu_set_gpr(decoded.rN, address);

  return TIMING_DEFAULT;
}

// STM - Store multiple registers to the stack
u32 stm() {
  u32 numStored = 0;
  u32 address = cpu_get_gpr(decoded.rN);

  for (int i = 0; i < 8; ++i) {
    int mask = 1 << i;
    if (decoded.reg_list & mask) {
      if (i == decoded.rN && numStored == 0) {
        fprintf(stderr, "Error: Malformed instruction!\n");
        sim_exit(1);
      }

      u32 data = cpu_get_gpr(i);
      simStoreWord(address, data);
      address += 4;
      ++numStored;
    }
  }

  cpu_set_gpr(decoded.rN, address);

  return TIMING_DEFAULT;
}

///--- Stack operations --------------------------------------------///

// Pop multiple reg values from the stack and update SP
u32 pop() {
  u32 numLoaded = 0;
  u32 address = cpu_get_sp();
  u32 exceptionReturnAddress = 0;
  u32 extraCycles = TIMING_DEFAULT;

  for (int i = 0; i < 16; ++i) {
    int mask = 1 << i;
    if (decoded.reg_list & mask) {
      u32 data = 0;
      simLoadWord(address, &data);
      cpu_set_gpr(i, data);
      ++numLoaded;
      if (i == 15) {  // PC is target
        takenBranch = 1;
        extraCycles = TIMING_POP_AND_RETURN;
        if ((data & 0xf0000000) == 0xf0000000) {
          // This is an exception return
          exceptionReturnAddress = data;
        }
      }
      address += 4;
    }

    // Skip constant 0s
    if (i == 7) i = 14;
  }

  cpu_set_sp(address);
  if (exceptionReturnAddress != 0) {
    exception_return_cb(exceptionReturnAddress);
  }

  return extraCycles;
}

// Push multiple reg values to the stack and update SP
u32 push() {
  u32 numStored = 0;
  u32 address = cpu_get_sp();

  for (int i = 14; i >= 0; --i) {
    int mask = 1 << i;
    if (decoded.reg_list & mask) {
      address -= 4;
      u32 data = cpu_get_gpr(i);
      simStoreWord(address, data);
      ++numStored;
    }

    // Skip constant 0s
    if (i == 14) i = 8;
  }

  cpu_set_sp(address);

  return TIMING_DEFAULT;
}

///--- Single load operations --------------------------------------------///

// LDR - Load from offset from register
u32 ldr_i() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = zeroExtend32(decoded.imm << 2);
  u32 effectiveAddress = base + offset;

  u32 result = 0;
  simLoadWord(effectiveAddress, &result);

  cpu_set_gpr(decoded.rD, result);

  return TIMING_DEFAULT;
}

// LDR - Load from offset from SP
u32 ldr_sp() {
  u32 base = cpu_get_sp();
  u32 offset = zeroExtend32(decoded.imm << 2);
  u32 effectiveAddress = base + offset;

  u32 result = 0;
  simLoadWord(effectiveAddress, &result);

  cpu_set_gpr(decoded.rD, result);

  return TIMING_DEFAULT;
}

// LDR - Load from offset from PC
u32 ldr_lit() {
  u32 base = cpu_get_pc() & 0xFFFFFFFC;
  u32 offset = zeroExtend32(decoded.imm << 2);
  u32 effectiveAddress = base + offset;

  u32 result = 0;
  simLoadWord(effectiveAddress, &result);

  cpu_set_gpr(decoded.rD, result);

  return TIMING_DEFAULT;
}

// LDR - Load from an offset from a reg based on another reg value
u32 ldr_r() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  u32 effectiveAddress = base + offset;

  u32 result = 0;
  simLoadWord(effectiveAddress, &result);

  cpu_set_gpr(decoded.rD, result);

  return TIMING_DEFAULT;
}

// LDRB - Load byte from offset from register
u32 ldrb_i() {
  u32 result = 0;
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = zeroExtend32(decoded.imm);
  simLoadByte(base + offset, &result);
  result = zeroExtend32(result & 0xFF);
  cpu_set_gpr(decoded.rD, result);
  return TIMING_DEFAULT;
}

// LDRB - Load byte from an offset from a reg based on another reg value
u32 ldrb_r() {
  u32 result = 0;
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  simLoadByte(base + offset, &result);
  result = zeroExtend32(result & 0xFF);
  cpu_set_gpr(decoded.rD, result);
  return TIMING_DEFAULT;
}

// LDRH - Load halfword from offset from register
u32 ldrh_i() {
  u32 result = 0;
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = zeroExtend32(decoded.imm << 1);
  u32 effectiveAddress = (base + offset) & (~1u);
  simLoadHalfWord(effectiveAddress, &result);
  result = zeroExtend32(result & 0xFFFF);
  cpu_set_gpr(decoded.rD, result);
  return TIMING_DEFAULT;
}

// LDRH - Load halfword from an offset from a reg based on another reg value
u32 ldrh_r() {
  u32 result = 0;
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  u32 effectiveAddress = (base + offset) & (~1u);
  simLoadHalfWord(effectiveAddress, &result);
  result = zeroExtend32(result & 0xFFFF);
  cpu_set_gpr(decoded.rD, result);
  return TIMING_DEFAULT;
}

// LDRSB - Load signed byte from an offset from a reg based on another reg value
u32 ldrsb_r() {
  u32 result = 0;
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  simLoadByte(base + offset, &result);
  result = signExtend32(result & 0xFF, 8);
  cpu_set_gpr(decoded.rD, result);
  return TIMING_DEFAULT;
}

// LDRSH - Load signed halfword from an offset from a reg based on another reg
// value
u32 ldrsh_r() {
  u32 result = 0;
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  u32 effectiveAddress = (base + offset) & (~1u);
  simLoadHalfWord(effectiveAddress, &result);
  result = signExtend32(result & 0xFFFF, 16);
  cpu_set_gpr(decoded.rD, result);
  return TIMING_DEFAULT;
}

///--- Single store operations --------------------------------------------///

// STR - Store to offset from register
u32 str_i() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = zeroExtend32(decoded.imm << 2);
  u32 effectiveAddress = base + offset;

  simStoreWord(effectiveAddress, cpu_get_gpr(decoded.rD));
  return TIMING_DEFAULT;
}

// STR - Store to offset from SP
u32 str_sp() {
  u32 base = cpu_get_sp();
  u32 offset = zeroExtend32(decoded.imm << 2);
  u32 effectiveAddress = base + offset;

  simStoreWord(effectiveAddress, cpu_get_gpr(decoded.rD));

  return TIMING_DEFAULT;
}

// STR - Store to an offset from a reg based on another reg value
u32 str_r() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  u32 effectiveAddress = base + offset;

  simStoreWord(effectiveAddress, cpu_get_gpr(decoded.rD));

  return TIMING_DEFAULT;
}

// STRB - Store byte to offset from register
u32 strb_i() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = zeroExtend32(decoded.imm);
  simStoreByte(base + offset, cpu_get_gpr(decoded.rD) & 0xFF);
  return TIMING_DEFAULT;
}

// STRB - Store byte to an offset from a reg based on another reg value
u32 strb_r() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  simStoreByte(base + offset, cpu_get_gpr(decoded.rD) & 0xFF);
  return TIMING_DEFAULT;
}

// STRH - Store halfword to offset from register
u32 strh_i() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = zeroExtend32(decoded.imm << 1);
  u32 effectiveAddress = (base + offset) & (~1u);
  u32 data = cpu_get_gpr(decoded.rD) & 0xFFFF;
  simStoreHalfWord(effectiveAddress, data);
  return TIMING_DEFAULT;
}

// STRH - Store halfword to an offset from a reg based on another reg value
u32 strh_r() {
  u32 base = cpu_get_gpr(decoded.rN);
  u32 offset = cpu_get_gpr(decoded.rM);
  u32 effectiveAddress = (base + offset) & (~1u);
  u32 data = cpu_get_gpr(decoded.rD) & 0xFFFF;
  simStoreHalfWord(effectiveAddress, data);
  return TIMING_DEFAULT;
}
