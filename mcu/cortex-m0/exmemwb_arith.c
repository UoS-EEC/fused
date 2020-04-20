/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#include <assert.h>
#include <stdlib.h>
#include "decode.h"
#include "exmemwb.h"

///--- Add operations --------------------------------------------///

// ADCS - add with carry and update flags
u32 adcs() {
  u32 opA = cpu_get_gpr(decoded.rD);
  u32 opB = cpu_get_gpr(decoded.rM);
  u32 result = opA + opB + cpu_get_flag_c();

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, cpu_get_flag_c());
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

// ADD - add small immediate to a register and update flags
u32 adds_i3() {
  u32 opA = cpu_get_gpr(decoded.rN);
  u32 opB = zeroExtend32(decoded.imm);
  u32 result = opA + opB;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, 0);
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

// ADD - add large immediate to a register and update flags
u32 adds_i8() {
  u32 opA = cpu_get_gpr(decoded.rD);
  u32 opB = zeroExtend32(decoded.imm);
  u32 result = opA + opB;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, 0);
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

// ADD - add two registers and update flags
u32 adds_r() {
  u32 opA = cpu_get_gpr(decoded.rN);
  u32 opB = cpu_get_gpr(decoded.rM);
  u32 result = opA + opB;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, 0);
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

// ADD - add two registers, one or both high no flags
u32 add_r() {
  // Check for malformed instruction
  if (decoded.rD == 15 && decoded.rM == 15) {
    // UNPREDICTABLE
    fprintf(stderr, "Error: Instruction format error.\n");
    sim_exit(1);
  }

  u32 opA = cpu_get_gpr(decoded.rD);
  u32 opB = cpu_get_gpr(decoded.rM);
  u32 result = opA + opB;

  // If changing the PC, check that thumb mode maintained
  if (decoded.rD == GPR_PC)
    alu_write_pc(result);
  else
    cpu_set_gpr(decoded.rD, result);

  // Instruction takes two cycles when PC is the destination
  return (decoded.rD == GPR_PC) ? 2 : 1;
}

// ADD - add an immpediate to SP
u32 add_sp() {
  u32 opA = cpu_get_sp();
  u32 opB = zeroExtend32(decoded.imm << 2);
  u32 result = opA + opB;

  cpu_set_gpr(decoded.rD, result);

  return TIMING_DEFAULT;
}

// ADR - add an immpediate to PC
u32 adr() {
  u32 opA = cpu_get_pc();
  // Align PC to 4 bytes
  opA = opA & 0xFFFFFFFC;
  u32 opB = zeroExtend32(decoded.imm << 2);
  u32 result = opA + opB;

  cpu_set_gpr(decoded.rD, result);

  return TIMING_DEFAULT;
}

///--- Subtract operations --------------------------------------------///

u32 subs_i3() {
  u32 opA = cpu_get_gpr(decoded.rN);
  u32 opB = ~zeroExtend32(decoded.imm);
  u32 result = opA + opB + 1;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, 1);
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

u32 subs_i8() {
  u32 opA = cpu_get_gpr(decoded.rD);
  u32 opB = ~zeroExtend32(decoded.imm);
  u32 result = opA + opB + 1;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, 1);
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

u32 subs() {
  u32 opA = cpu_get_gpr(decoded.rN);
  u32 opB = ~cpu_get_gpr(decoded.rM);
  u32 result = opA + opB + 1;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, 1);
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

u32 sub_sp() {
  u32 opA = cpu_get_sp();
  u32 opB = ~zeroExtend32(decoded.imm << 2);
  u32 result = opA + opB + 1;

  cpu_set_sp(result);

  return TIMING_DEFAULT;
}

u32 sbcs() {
  u32 opA = cpu_get_gpr(decoded.rD);
  u32 opB = ~cpu_get_gpr(decoded.rM);
  u32 result = opA + opB + cpu_get_flag_c();

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, cpu_get_flag_c());
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

u32 rsbs() {
  u32 opA = 0;
  u32 opB = ~(cpu_get_gpr(decoded.rN));
  u32 result = opA + opB + 1;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);
  do_cflag(opA, opB, 1);
  do_vflag(opA, opB, result);

  return TIMING_DEFAULT;
}

///--- Multiply operations --------------------------------------------///

// MULS - multiply the source and destination and store 32-bits in dest
// Does not update carry or overflow: simple mult
u32 muls() {
  u32 opA = cpu_get_gpr(decoded.rD);
  u32 opB = cpu_get_gpr(decoded.rM);
  u32 result = opA * opB;

  cpu_set_gpr(decoded.rD, result);

  do_nflag(result);
  do_zflag(result);

  return TIMING_MULTIPLY;
}
