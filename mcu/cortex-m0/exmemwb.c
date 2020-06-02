/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>
#include "decode.h"
#include "exmemwb.h"

u16 insn;

#if HOOK_GPR_ACCESSES
u32 cpu_get_gpr(u32 gpr) {
  gprReadHooks[gpr]();
  return cpu.gpr[gpr];
}

void cpu_set_gpr(u32 gpr, u32 value) {
  gprWriteHooks[gpr]();
  cpu.gpr[gpr] = value;
}
#endif

void do_cflag(u32 a, u32 b, u32 carry) {
  u32 result;

  result = (a & 0x7FFFFFFF) + (b & 0x7FFFFFFF) + carry;  // carry in
  result = (result >> 31) + (a >> 31) + (b >> 31);       // carry out
  cpu_set_flag_c(result >> 1);
}

u32 adcs(void);
u32 adds_i3(void);
u32 adds_i8(void);
u32 adds_r(void);
u32 add_r(void);
u32 add_sp(void);
u32 adr(void);
u32 subs_i3(void);
u32 subs_i8(void);
u32 subs(void);
u32 sub_sp(void);
u32 sbcs(void);
u32 rsbs(void);
u32 muls(void);
u32 cmn(void);
u32 cmp_i(void);
u32 cmp_r(void);
u32 tst(void);
u32 b(void);
u32 b_c(void);
u32 blx(void);
u32 bx(void);
u32 bl(void);
u32 ands(void);
u32 bics(void);
u32 eors(void);
u32 orrs(void);
u32 mvns(void);
u32 asrs_i(void);
u32 asrs_r(void);
u32 lsls_i(void);
u32 lsrs_i(void);
u32 lsls_r(void);
u32 lsrs_r(void);
u32 rors(void);
u32 ldm(void);
u32 stm(void);
u32 pop(void);
u32 push(void);
u32 ldr_i(void);
u32 ldr_sp(void);
u32 ldr_lit(void);
u32 ldr_r(void);
u32 ldrb_i(void);
u32 ldrb_r(void);
u32 ldrh_i(void);
u32 ldrh_r(void);
u32 ldrsb_r(void);
u32 ldrsh_r(void);
u32 str_i(void);
u32 str_sp(void);
u32 str_r(void);
u32 strb_i(void);
u32 strb_r(void);
u32 strh_i(void);
u32 strh_r(void);
u32 movs_i(void);
u32 mov_r(void);
u32 movs_r(void);
u32 msr(void);
u32 mrs(void);
u32 sxtb(void);
u32 sxth(void);
u32 uxtb(void);
u32 uxth(void);
u32 rev(void);
u32 rev16(void);
u32 revsh(void);
u32 cps(void);

u32 exmemwb_error() {
  fprintf(stderr, "Error: Unsupported instruction: Unable to execute\n");
  sim_exit(1);
  return 0;
}

// Execute functions that require more opcode bits than the first 6
u32 (*executeJumpTable6[2])(void) = {
    adds_r, /* 060 - 067 */
    subs    /* 068 - 06F */
};

u32 entry6(void) { return executeJumpTable6[(insn >> 9) & 0x1](); }

u32 (*executeJumpTable7[2])(void) = {
    adds_i3, /* (070 - 077) */
    subs_i3  /* (078 - 07F) */
};

u32 entry7(void) { return executeJumpTable7[(insn >> 9) & 0x1](); }

u32 (*executeJumpTable16[16])(void) = {
    ands, eors, lsls_r, lsrs_r,        asrs_r, adcs, sbcs, rors,
    tst,  rsbs, cmp_r,  exmemwb_error, orrs,   muls, bics, mvns};

u32 entry16(void) { return executeJumpTable16[(insn >> 6) & 0xF](); }

u32 (*executeJumpTable17[8])(void) = {
    add_r,        /* (110 - 113) */
    add_r, cmp_r, /* (114 - 117) */
    cmp_r, mov_r, /* (118 - 11B) */
    mov_r, bx,    /* (11C - 11D) */
    blx           /* (11E - 11F) */
};

u32 entry17(void) { return executeJumpTable17[(insn >> 7) & 0x7](); }

u32 (*executeJumpTable20[2])(void) = {
    str_r, /* (140 - 147) */
    strh_r /* (148 - 14F) */
};

u32 entry20(void) { return executeJumpTable20[(insn >> 9) & 0x1](); }

u32 (*executeJumpTable21[2])(void) = {
    strb_r, /* (150 - 157) */
    ldrsb_r /* (158 - 15F) */
};

u32 entry21(void) { return executeJumpTable21[(insn >> 9) & 0x1](); }

u32 (*executeJumpTable22[2])(void) = {
    ldr_r, /* (160 - 167) */
    ldrh_r /* (168 - 16F) */
};

u32 entry22(void) { return executeJumpTable22[(insn >> 9) & 0x1](); }

u32 (*executeJumpTable23[2])(void) = {
    ldrb_r, /* (170 - 177) */
    ldrsh_r /* (178 - 17F) */
};

u32 entry23(void) { return executeJumpTable23[(insn >> 9) & 0x1](); }

u32 (*executeJumpTable44[16])(void) = {
    add_sp,                /* (2C0 - 2C1) */
    add_sp,        sub_sp, /* (2C2 - 2C3) */
    sub_sp,        exmemwb_error, exmemwb_error, exmemwb_error, exmemwb_error,
    sxth,          sxtb,          uxth,          uxtb,          exmemwb_error,
    exmemwb_error, exmemwb_error, exmemwb_error};

u32 entry44(void) { return executeJumpTable44[(insn >> 6) & 0xF](); }

u32 (*executeJumpTable45[2])(void) = {push, cps};

u32 entry45(void) { return executeJumpTable45[(insn >> 9) & 0x1](); }

u32 (*executeJumpTable46[16])(void) = {
    exmemwb_error, exmemwb_error, exmemwb_error, exmemwb_error,
    exmemwb_error, exmemwb_error, exmemwb_error, exmemwb_error,
    rev,           rev16,         exmemwb_error, exmemwb_error,
    exmemwb_error, exmemwb_error, exmemwb_error, exmemwb_error};

u32 entry46(void) { return executeJumpTable46[(insn >> 6) & 0xF](); }

u32 (*executeJumpTable47[2])(void) = {pop, /* (2F0 - 2F7) */
                                      // breakpoint /* (2F8 - 2FB) */
                                      // SEV?
                                      exmemwb_error};

u32 entry47(void) { return executeJumpTable47[(insn >> 9) & 0x1](); }

u32 entry55(void) {
  if ((insn & 0x0300) != 0x0300) {
    return b_c();
  } else if ((insn & 0xbe00) == 0xbe00) {
    // Breakpoint instruction
    fprintf(stderr, "BKPT not implemented.");
  } else {
    fprintf(stderr, "Instruction 0x%04x not implemented.", insn);
  }
  return exmemwb_error();
}

u32 entry6061(void) {
  if (decoded.imm == 0x80000000) {
    if ((insn & 0xf3e0) == 0xf3e0) {  // MRS
      return mrs();
    } else if ((insn & 0xf380) == 0xf380) {  // MSR
      return msr();
    }
  } else {
    return bl();
  }
}

u32 (*executeJumpTable[64])() = {lsls_i,
                                 lsls_i,
                                 lsrs_i,
                                 lsrs_i,
                                 asrs_i,
                                 asrs_i,
                                 entry6, /* 6 */
                                 entry7, /* 7 */
                                 movs_i,
                                 movs_i,
                                 cmp_i,
                                 cmp_i,
                                 adds_i8,
                                 adds_i8,
                                 subs_i8,
                                 subs_i8,
                                 entry16, /* 16 */
                                 entry17, /* 17 */
                                 ldr_lit,
                                 ldr_lit,
                                 entry20, /* 20 */
                                 entry21, /* 21 */
                                 entry22, /* 22 */
                                 entry23, /* 23 */
                                 str_i,
                                 str_i,
                                 ldr_i,
                                 ldr_i,
                                 strb_i,
                                 strb_i,
                                 ldrb_i,
                                 ldrb_i,
                                 strh_i,
                                 strh_i,
                                 ldrh_i,
                                 ldrh_i,
                                 str_sp,
                                 str_sp,
                                 ldr_sp,
                                 ldr_sp,
                                 adr,
                                 adr,
                                 add_sp,
                                 add_sp,
                                 entry44, /* 44 */
                                 entry45, /* 45 */
                                 entry46, /* 46 */
                                 entry47, /* 47 */
                                 stm,
                                 stm,
                                 ldm,
                                 ldm,
                                 b_c,
                                 b_c,
                                 b_c,
                                 entry55, /* 55 */
                                 b,
                                 b,
                                 exmemwb_error,
                                 exmemwb_error,
                                 entry6061, /* 60 bl or mrs/msr */
                                 entry6061, /* 61 bl or undef */
                                 exmemwb_error,
                                 exmemwb_error};

size_t exwbmem(const u16 pInsn) {
  insn = pInsn;
  return executeJumpTable[pInsn >> 10]();
}
