/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <common.h>
#include <msp430fr5994.h>
#include <stdio.h>

static uint8_t source[1024] __attribute__((section(SOURCE_SECTION)));
static uint8_t dest[1024] __attribute__((section(DEST_SECTION)));

__attribute__((section(CODE_SECTION))) void mymemcpy(uint8_t *dst, uint8_t *src,
                                                     size_t len) {
    while (len--) {
        *dst++ = *src++;
    }
}

__attribute__((naked, section(CODE_SECTION))) void mymemcpy_asm(uint8_t *dst,
                                                                uint8_t *src,
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

__attribute__((section(".ramtext"))) int main(int argc, char **argv) {
    cs_init();
    gpio_init();
    while (1) {
        cache_flush();
        // settle
        WAIT P1OUT |= BIT2;
        for (int i = 0; i < 100; i++) {
            RUN_TEST(dest, source, sizeof(source));
        }
        P1OUT &= ~BIT2;
        WAIT
    }
}

