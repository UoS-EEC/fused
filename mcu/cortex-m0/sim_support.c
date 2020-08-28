/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#include <stdlib.h>
#include "exmemwb.h"
#include "sim_support.h"

bool takenBranch = 0;

// Callbacks
void (*write_cb)(const uint32_t, uint8_t *const, size_t);
void (*read_cb)(const uint32_t, uint8_t *const, size_t);
void (*consume_cycles_cb)(const size_t);
void (*exception_return_cb)(const uint32_t);
uint16_t (*next_pipeline_instr_cb)(void);

// Set callback to write data to memory
void cpu_set_write_memory_cb(void (*fptr)(const uint32_t, uint8_t *const,
                                          size_t)) {
  write_cb = fptr;
}

// Set callback to read data from memory
void cpu_set_read_memory_cb(void (*fptr)(const uint32_t, uint8_t *const,
                                         size_t)) {
  read_cb = fptr;
}

// Set callback to consume clock cycles / simulation time
void cpu_set_consume_cycles_cb(void (*fptr)(const size_t)) {
  consume_cycles_cb = fptr;
}

// Set callback for exception return
void cpu_set_exception_return_cb(void (*fptr)(const uint32_t)) {
  exception_return_cb = fptr;
}

// Set callback for exception return
void cpu_set_next_pipeline_instr_cb(uint16_t (*fptr)(void)) {
  next_pipeline_instr_cb = fptr;
}

uint32_t pack32(const uint8_t *const data) {
#ifdef TARGET_BIG_ENDIAN
  return ((uint32_t)data[0] << 24 | (uint32_t)data[1] << 16 |
          (uint32_t)data[2] << 8 | (uint32_t)data[3] << 0);
#elif defined(TARGET_LITTLE_ENDIAN)
  return ((uint32_t)data[3] << 24 | (uint32_t)data[2] << 16 |
          (uint32_t)data[1] << 8 | (uint32_t)data[0] << 0);
#endif
}

void unpack32(uint8_t *const out, const uint32_t in) {
#ifdef TARGET_BIG_ENDIAN
  out[0] = (0xff000000 & in) >> 24;
  out[1] = (0x00ff0000 & in) >> 16;
  out[2] = (0x0000ff00 & in) >> 8;
  out[3] = (0x000000ff & in) >> 0;
#elif defined(TARGET_LITTLE_ENDIAN)
  out[3] = (0xff000000 & in) >> 24;
  out[2] = (0x00ff0000 & in) >> 16;
  out[1] = (0x0000ff00 & in) >> 8;
  out[0] = (0x000000ff & in) >> 0;
#endif
}

uint16_t pack16(const uint8_t *const data) {
#ifdef TARGET_BIG_ENDIAN
  return ((uint16_t)data[0] << 8 | (uint16_t)data[1] << 0);
#elif defined(TARGET_LITTLE_ENDIAN)
  return ((uint16_t)data[1] << 8 | (uint16_t)data[0] << 0);
#endif
}

void unpack16(uint8_t *const out, const uint16_t in) {
#ifdef TARGET_BIG_ENDIAN
  out[0] = (0x0000ff00 & in) >> 8;
  out[1] = (0x000000ff & in) >> 0;
#elif defined(TARGET_LITTLE_ENDIAN)
  out[1] = (0x0000ff00 & in) >> 8;
  out[0] = (0x000000ff & in) >> 0;
#endif
}

#if HOOK_GPR_ACCESSES
void do_nothing(void) { ; }

void report_sp(void) {
  if (cpu_get_sp() < 0X40010000) {
    fprintf(stderr, "SP crosses heap: 0x%8.8X\n", cpu_get_sp());
    fprintf(stderr, "PC: 0x%8.8X\n", cpu_get_pc());
  }
}

void (*gprReadHooks[16])(void) = {
    do_nothing, do_nothing, do_nothing, do_nothing, do_nothing, do_nothing,
    do_nothing, do_nothing, do_nothing, do_nothing, do_nothing, do_nothing,
    do_nothing, do_nothing, do_nothing, do_nothing};

void (*gprWriteHooks[16])(void) = {
    do_nothing, do_nothing, do_nothing, do_nothing, do_nothing, do_nothing,
    do_nothing, do_nothing, do_nothing, do_nothing, do_nothing, do_nothing,
    do_nothing, do_nothing, report_sp,  do_nothing};
#endif

// load 16-bit instruction
char simLoadInsn(u32 address, u16 *value) {
  uint8_t tmp[2];
  read_cb(address & (~1u), tmp, 2);  // Mask thumb-bit & read
  *value = pack16(tmp);
  return 0;
}

char simLoadWord(u32 address, u32 *value) {
  uint8_t tmp[4];
  read_cb(address, tmp, 4);
  *value = pack32(tmp);
  return 0;
}

char simLoadHalfWord(u32 address, u32 *value) {
  uint8_t tmp[2];
  read_cb(address, tmp, 2);
  *value = pack16(tmp);
  return 0;
}

char simLoadByte(u32 address, u32 *value) {
  uint8_t tmp[1];
  read_cb(address, tmp, 1);
  *value = tmp[0];
  return 0;
}

char simStoreWord(u32 address, u32 value) {
  if ((address & 0x3) != 0)  // Thumb-mode requires LSB = 1
  {
    fprintf(stderr, "Unalinged data memory write: 0x%8.8X\n", address);
    sim_exit(1);
  }

  // Unpack data
  uint8_t data[4];
  unpack32(data, value);
  write_cb(address, data, 4);
  return 0;
}

char simStoreHalfWord(u32 address, u16 value) {
  if ((address & 0x1) != 0) {
    fprintf(stderr, "Unalinged half word write: 0x%8.8X\n", address);
    sim_exit(1);
  }

  // Unpack data
  uint8_t data[2];
  unpack16(data, value);
  write_cb(address, data, 2);
  return 0;
}

char simStoreByte(u32 address, u8 value) {
  write_cb(address, &value, 1);
  return 0;
}

void sim_exit(int i) {
  fprintf(stderr, "Simulator exiting from CM0 model...\n");
  exit(i);
}
