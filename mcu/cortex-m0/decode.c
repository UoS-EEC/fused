/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#include "decode.h"
#include "exmemwb.h"
#include <stdlib.h>

// Holds the result of decoding
// Passed to exmemwb stage
DECODE_RESULT decoded;

// Various decodings
void decode_3lo(const u16 pInsn) {
  decoded.rD = pInsn & 0x7;
  decoded.rN = (pInsn >> 3) & 0x7;
  decoded.rM = (pInsn >> 6) & 0x7;
}

void decode_2loimm5(const u16 pInsn) {
  decoded.rD = pInsn & 0x7;
#if DECODE_SAFE
  decoded.rM = (pInsn >> 3) & 0x7; // Just to be safe
#endif
  decoded.rN = (pInsn >> 3) & 0x7;
  decoded.imm = (pInsn >> 6) & 0x1F;
}

void decode_2loimm3(const u16 pInsn) {
  decoded.rD = pInsn & 0x7;
#if DECODE_SAFE
  decoded.rM = (pInsn >> 3) & 0x7; // Just to be safe
#endif
  decoded.rN = (pInsn >> 3) & 0x7;
  decoded.imm = (pInsn >> 6) & 0x7;
}

void decode_2lo(const u16 pInsn) {
  decoded.rD = pInsn & 0x7;
  decoded.rM = (pInsn >> 3) & 0x7;
  decoded.rN = (pInsn >> 3) & 0x7;
}

void decode_imm8lo(const u16 pInsn) {
  decoded.rD = (pInsn >> 8) & 0x7;
#if DECODE_SAFE
  decoded.rM = decoded.rD; // Just to be safe
  decoded.rN = decoded.rD; // Just to be safe
#endif
  decoded.imm = pInsn & 0xFF;
}

void decode_imm8(const u16 pInsn) { decoded.imm = pInsn & 0xFF; }

void decode_imm8c(const u16 pInsn) {
  decoded.imm = pInsn & 0xFF;
  decoded.cond = (pInsn >> 8) & 0xF;
}

void decode_imm7(const u16 pInsn) {
  decoded.rD = GPR_SP;
  decoded.imm = pInsn & 0x7F;
}

void decode_imm11(const u16 pInsn) { decoded.imm = pInsn & 0x7FF; }

void decode_reglistlo(const u16 pInsn) {
  decoded.rN = (pInsn >> 8) & 0x7;
  decoded.reg_list = pInsn & 0xFF;
}

void decode_pop(const u16 pInsn) {
  decoded.reg_list = ((pInsn & 0x100) << 7) | (pInsn & 0xFF);
}

void decode_push(const u16 pInsn) {
  decoded.reg_list = (pInsn & 0xFF) | ((pInsn & 0x100) << 6);
}

void decode_bl(const u16 pInsn) {
  const u32 BLMASK = 0xf000d000;
  const u32 MSRMASK = 0xf3808000;
  const u32 MRSMASK = 0xf3e08000;
  u16 secondHalf = next_pipeline_instr_cb();
  u32 wholeInsn = ((u32) pInsn << 16) | secondHalf;

  if ((wholeInsn & BLMASK) == BLMASK) { // Branch & link (BL)
    u32 S = (pInsn >> 10) & 0x1;
    u32 J1 = (secondHalf >> 13) & 0x1;
    u32 J2 = (secondHalf >> 11) & 0x1;
    u32 I1 = ~(J1 ^ S) & 0x1;
    u32 I2 = ~(J2 ^ S) & 0x1;
    u32 imm10 = pInsn & 0x3FF;
    u32 imm11 = secondHalf & 0x7FF;
    decoded.imm = (S << 23) | (I1 << 22) | (I2 << 21) | (imm10 << 11) | imm11;
  } else if ((wholeInsn & MRSMASK) == MRSMASK) { // Move to Register from Special register
    decoded.rM = wholeInsn & 0x000000ff;
    decoded.rD = (wholeInsn & 0x00000f00) >> 8;
    decoded.imm = 0x80000000; // Hack to signal MRS/MSR (impossible value for bl)
  } else if ((wholeInsn & MSRMASK) == MSRMASK) { // Move to Special register from Register
    decoded.rD = wholeInsn & 0x000000ff;
    decoded.rM = (wholeInsn & 0x000f0000) >> 16;
    decoded.imm = 0x80000000; // Hack to signal MRS/MSR (impossible value for bl)
  } else {
    fprintf(stderr, "Unrecognized instruction %08x, exiting.", wholeInsn);
    sim_exit(1);
  }
}

void decode_1all(const u16 pInsn) { decoded.rM = (pInsn >> 3) & 0xF; }

void decode_mov_r(const u16 pInsn) {
  decoded.rD = (pInsn & 0x7) | ((pInsn & 0x80) >> 4);
  decoded.rN = decoded.rD;
  decoded.rM = (pInsn >> 3) & 0xF;
}

// Stop simulation if we cannot decode the instruction
void decode_error(const u16 pInsn) {
  fprintf(stderr,
          "Error: Malformed instruction: Unable to decode: 0x%4.4X at 0x%08X\n",
          pInsn, cpu_get_pc() - 4);
  sim_exit(1);
}

// Decode functions that require more opcode bits than the first 6
void (*decodeJumpTable17[4])(const u16 pInsn) = {
    decode_mov_r,               /* 01_0001_0XXX (110 - 117) */
    decode_mov_r, decode_mov_r, /* 01_0001_10XX (118 - 11B) */
    decode_1all                 /* 01_0001_11XX (11C - 11F) */
};

void (*decodeJumpTable44[4])(const u16 pInsn) = {
    decode_imm7,              /* 10_1100_00XX (2C0 - 2C3) */
    decode_error, decode_2lo, /* 10_1100_10XX (2C8 - 2CB) */
    decode_error};

void (*decodeJumpTable47[4])(const u16 pInsn) = {
    decode_pop,              /* 10_1111_0XXX (2F0 - 2F7) */
    decode_pop, decode_imm8, /* 10_1111_10XX (2F8 - 2FB) */
    decode_error};

void decode_17(const u16 pInsn) {
  decodeJumpTable17[(pInsn >> 8) & 0x3](pInsn);
}
void decode_44(const u16 pInsn) {
  decodeJumpTable44[(pInsn >> 8) & 0x3](pInsn);
}
void decode_47(const u16 pInsn) {
  decodeJumpTable47[(pInsn >> 8) & 0x3](pInsn);
}

// Use a table of function pointers indexed by the instruction
// to make decoding fast
// Indices 16, 17, 44, 47, 60, and 62 have multiple conflicting
// decodings that need to be resolved outside the jump table
void (*decodeJumpTable[64])(const u16 pInsn) = {
    decode_2loimm5,   decode_2loimm5,   decode_2loimm5,   decode_2loimm5,
    decode_2loimm5,   decode_2loimm5,   decode_3lo,       decode_2loimm3,
    decode_imm8lo,    decode_imm8lo,    decode_imm8lo,    decode_imm8lo,
    decode_imm8lo,    decode_imm8lo,    decode_imm8lo,    decode_imm8lo,
    decode_2lo, /* A5.2.2 - these are all decoded the same */
    decode_17,  /* 17 */
    decode_imm8lo,    decode_imm8lo,    decode_3lo,       decode_3lo,
    decode_3lo,       decode_3lo,       decode_2loimm5,   decode_2loimm5,
    decode_2loimm5,   decode_2loimm5,   decode_2loimm5,   decode_2loimm5,
    decode_2loimm5,   decode_2loimm5,   decode_2loimm5,   decode_2loimm5,
    decode_2loimm5,   decode_2loimm5,   decode_imm8lo,    decode_imm8lo,
    decode_imm8lo,    decode_imm8lo,    decode_imm8lo,    decode_imm8lo,
    decode_imm8lo,    decode_imm8lo,    decode_44, /* 44 */
    decode_push,      decode_2lo,       decode_47, /* 47 */
    decode_reglistlo, decode_reglistlo, decode_reglistlo, decode_reglistlo,
    decode_imm8c,     decode_imm8c,     decode_imm8c,     decode_imm8c,
    decode_imm11,     decode_imm11,     decode_error, /* 58 */
    decode_error,                                     /* 59 */
    decode_bl, /* Ignore other possible decodings */
    decode_bl, /* Ignore other possible decodings */
    decode_error,     decode_error};

// Decoding is a matter of indexing the decode jump table
// using the first 6 instruction opcode bits and then
// executing the function pointed to
// The decode functions update the global decode structure
void decode(const u16 pInsn) {
// Clear the values from the previous decode
#if DECODE_CLEAR
  decoded.rD = 0;
  decoded.rM = 0;
  decoded.rN = 0;
  decoded.imm = 0;
  decoded.cond = 0;
  decoded.reg_list = 0;
#endif

  decodeJumpTable[pInsn >> 10](pInsn);
}
