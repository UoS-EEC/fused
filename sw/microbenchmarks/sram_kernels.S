/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

.file "sram_kernels.S"
        .global kernel_registers
        .global kernel_sram
        .global kernel_sram_byte
        .global kernel_sram_wsingle
        .global kernel_sram_infloop
        .global kernel_fram_raw

.data
rpt:
    .word 6000

kernel_registers:
    mov &rpt, r12
    bis #4, &P1OUT
loop:
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    sub r4, r5
    add r4, r5
    add #-1, r12
    jnz loop
    nop
    bic #4, &P1OUT
    ret

.data
word_one:
    .word 1
word_two:
    .word 2

kernel_sram:
    mov &rpt, r12
    bis #4, &P1OUT
sram_loop:
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    mov &word_one, &word_two
    add #-1, r12
    jnz sram_loop
    nop
    bic #4, &P1OUT
    ret

.data
sramvar1:
    .word 0
.data
kernel_sram_infloop:
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    mov &sramvar1, r5
    jmp kernel_sram_infloop

.data
kernel_sram_wsingle:
    mov &rpt, r12
    mov #1, r5
    bis #4, P1OUT

loop_ksw:
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1
    mov r5, &sramvar1

    add #-1, r12
    jnz loop_ksw
    bic #4, &P1OUT
    ret

// Test read-write-read-write... to same location in fram
.text
fram_var:
    .word 0

.text
rpt_raw:
    .word 10000

.data
kernel_fram_raw:
    mov &rpt, r12
    mov #1, r5
    mov #fram_var, r6
    bis #4, P1OUT

loop_kfraw:
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5
    mov r5, 0(r6)
    mov 0(r6), r5

    add #-1, r12
    jnz loop_kfraw
    bic #4, &P1OUT
    ret
