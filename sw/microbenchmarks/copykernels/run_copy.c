/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <support.h>
#include <msp430fr5994.h>
#include <stdio.h>
#include <string.h>

static uint8_t source[1024] __attribute__((section(SOURCE_SECTION)));
static uint8_t dest[1024] __attribute__((section(DEST_SECTION)));

__attribute__((naked)) void memcpy_asm(uint8_t *dst, uint8_t *src,
                                         size_t len) {
    __asm__(
        " push r5\n"
        " mov #2, r5\n"    // r5 = word size
        " xor r15, r15\n"  // Clear r15
        " mov r14, r15\n"  // r15=len
        " and #1, r15\n"   // r15 = len%2
        " sub r15, r14\n"  // r14 = len - len%2
        "loopmemcpy:  \n"
        " mov.w @r13+, @r12\n"
        " add r5, r12 \n"
        " sub r5, r14 \n"
        " jnz loopmemcpy \n"
        " tst r15\n"
        " jz return\n"
        " mov.b @r13, @r12\n"  // move last byte
        "return:\n"
        " pop r5\n"
        " ret\n");
}

__attribute__((optimize("O1")))
int main(int argc, char **argv) {
    target_init();
    while (1) {
        cache_flush();
        indicate_workload_begin();
        for (volatile int i = 0; i < REPETITIONS; i++) {
            RUN_TEST(dest, source, sizeof(source));
        }
        indicate_workload_end();
        wait();
    }
}

