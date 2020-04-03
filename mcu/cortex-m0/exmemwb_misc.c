/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#include "decode.h"
#include "exmemwb.h"
#include <stdlib.h>

///--- Move operations -------------------------------------------///

// MOVS - write an immediate to the destination register
u32 movs_i() {
  u32 opA = zeroExtend32(decoded.imm);
  cpu_set_gpr(decoded.rD, opA);

  do_nflag(opA);
  do_zflag(opA);

  return 1;
}

// MOV - copy the source register value to the destination register
u32 mov_r() {
  u32 opA = cpu_get_gpr(decoded.rM);

  if (decoded.rD == GPR_PC)
    alu_write_pc(opA);
  else
    cpu_set_gpr(decoded.rD, opA);

  return 1;
}

// MOVS - copy the low source register value to the destination low register
u32 movs_r() {
  u32 opA = cpu_get_gpr(decoded.rM);
  cpu_set_gpr(decoded.rD, opA);

  do_nflag(opA);
  do_zflag(opA);

  return 1;
}

// MRS - Move to Register from Special register
u32 mrs() {
  int n = decoded.rM;
  int d = decoded.rD;
  int result = 0;

  switch ((n & 0xf8) >> 3) {
    case 0:
      if (n & 1u) {
        result |= cpu_get_ipsr();
      }
      if (n & 2u) {
        result |= cpu_get_gpr(d) & (~(1u<<24)); // T-bit read as 0
      }
      if ((n & 4u) == 0) {
        result |= cpu_get_apsr() & 0xf0000000;
      }
      break;
    case 1:
      if ((n & 7u) == 0) {
        fprintf(stderr, "MRS: Warning, returning current stack pointer, but "
            "main stack pointer was requested.\n");
        result = cpu_get_sp();
      } else if ((n & 7u) == 1) {
        fprintf(stderr, "MRS: Warning, returning current stack pointer, but "
            "process stack pointer was requested.\n");
        result = cpu_get_sp();
      }
      break;
    case 2:
      if ((n & 7u) == 0) {
        result = cpu.primask & 1u;
      } else if ((n & 7u) == 4u) {
        result = cpu.control & 3u;
      }
  }
  cpu_set_gpr(d, result);
  takenBranch = 1;
  return 1;
}

// MSR - Move to Special register from Register
u32 msr() {
  int n = decoded.rM;
  int nval = cpu_get_gpr(n);
  int d = decoded.rD;
  switch ((d & 0xf8) >> 3) {
    case 0:
      if (d & 0b100) {
        cpu_set_apsr(nval & 0xf8000000);
      }
      break;
    case 1:
      fprintf(stderr, "MSR: Warning, we're not checking privelege before "
          "setting SP.\n");
      if ((d & 7u) == 0) {
        fprintf(stderr, "MSR: Warning, setting current stack, should set main"
            " stack pointer.\n");
        // set sp_main
        cpu_set_sp(nval & (~3u));
        fprintf(stderr, "MSR: stack pointer set to 0x%08x.\n", nval & (~3u));

      } else if ((d & 7u) == 1) {
        fprintf(stderr, "MSR: Warning, setting current stack, should set "
            "process stack pointer.\n");
        // set sp_process
        cpu_set_sp(nval & (~3u));
      }
      break;
    case 2:
      fprintf(stderr, "MSR: Warning, we're not checking privelege before "
          "setting CONTROL.\n");
      if ((d & 7u) == 0) {
          cpu.primask = nval & 1u;
      } else if (((d & 7u) == 4u) && cpu_mode_is_thread()) {
        cpu.control = (cpu.control & (~1u)) | (nval & (~1u)); // nPRIV
        cpu.control = (cpu.control & (~2u)) | (nval & (~2u)); // SPSEL
      }
      break;
    default:
      fprintf(stderr, "MSR: Unrecognized instruction.\n");
      break;
  }
  takenBranch = 1;
  return 1;
}

///--- Bit twiddling operations -------------------------------------------///

// SXTB - Sign extend a byte to a word
u32 sxtb() {
  u32 result = 0xFF & cpu_get_gpr(decoded.rM);
  result = (result & 0x80) != 0 ? (result | 0xFFFFFF00) : result;

  cpu_set_gpr(decoded.rD, result);

  return 1;
}

// SXTH - Sign extend a halfword to a word
u32 sxth() {
  u32 result = 0xFFFF & cpu_get_gpr(decoded.rM);
  result = (result & 0x8000) != 0 ? (result | 0xFFFF0000) : result;

  cpu_set_gpr(decoded.rD, result);

  return 1;
}

// UXTB - Extend a byte to a word
u32 uxtb() {
  u32 result = 0xFF & cpu_get_gpr(decoded.rM);
  cpu_set_gpr(decoded.rD, result);

  return 1;
}

// UXTH - Extend a halfword to a word
u32 uxth() {
  u32 result = 0xFFFF & cpu_get_gpr(decoded.rM);
  cpu_set_gpr(decoded.rD, result);

  return 1;
}

// REV - Reverse ordering of bytes in a word
u32 rev() {
  u32 opA = cpu_get_gpr(decoded.rM);
  u32 result = opA << 24;
  result |= (opA << 8) & 0xFF0000;
  result |= (opA >> 8) & 0xFF00;
  result |= (opA >> 24);

  cpu_set_gpr(decoded.rD, result);

  return 1;
}

// REV16 - Reverse ordering of bytes in a packed halfword
u32 rev16() {
  u32 opA = cpu_get_gpr(decoded.rM);
  u32 result = (opA << 8) & 0xFF000000;
  result |= (opA >> 8) & 0xFF0000;
  result |= (opA << 8) & 0xFF00;
  result |= (opA >> 8) & 0xFF;

  cpu_set_gpr(decoded.rD, result);

  return 1;
}

// REVSH - Reverse ordering of bytes in a signed halfword
u32 revsh() {
  u32 opA = cpu_get_gpr(decoded.rM);
  u32 result = (opA & 0x8) != 0 ? (0xFFFFFF00 | opA) : (0xFF & opA);
  result <<= 8;
  result |= (opA >> 8) & 0xFF;

  cpu_set_gpr(decoded.rD, result);

  return 1;
}
