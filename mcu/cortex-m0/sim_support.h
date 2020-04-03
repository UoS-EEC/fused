/*
 * Copyright (c) 2016-2020, Matthew Hicks and Contributors.
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef SIMSUPPORT_HEADER
#define SIMSUPPORT_HEADER

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define WATCHPOINT_ADDR 0x80000010

typedef __uint32_t u32;
typedef __uint64_t u64;
typedef __int32_t i32;
typedef __int64_t i64;
typedef __uint8_t u8;
typedef __uint16_t u16;
typedef struct CPU cpu_t;

// callbacks for memory access
extern void (*write_cb)(const uint32_t, uint8_t *const, size_t);
extern void (*read_cb)(const uint32_t, uint8_t *const, size_t);
extern void (*consume_cycles_cb)(const size_t);
extern void (*exception_return_cb)(const uint32_t);
extern uint16_t (*next_pipeline_instr_cb)(void);

void cpu_set_write_memory_cb(void (*fptr)(const uint32_t, uint8_t *const,
                                          size_t));
void cpu_set_read_memory_cb(void (*fptr)(const uint32_t, uint8_t *const,
                                         size_t));
void cpu_set_consume_cycles_cb(void (*fptr)(const size_t));

void cpu_set_exception_return_cb(void (*fptr)(const uint32_t));

void cpu_set_next_pipeline_instr_cb(uint16_t (*fptr)(void));

// Core CPU components
extern bool takenBranch;    // Informs fetch that previous instruction caused a
                            // control flow change
extern void sim_exit(int);  // All sim ends lead through here
char simLoadInsn(u32 address,
                 u16 *value);  // All memory accesses once simulation starts
                               // should be through these interfaces
char simLoadData(u32 address, u32 *value);
char simStoreData(u32 address, u32 value);

// Hooks to run code every time a GPR is accessed
#define HOOK_GPR_ACCESSES 0

#if HOOK_GPR_ACCESSES
void do_nothing(void);
void report_sp(void);
void (*gprReadHooks[16])(void);
void (*gprWriteHooks[16])(void);
#endif
#endif
