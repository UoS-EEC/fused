/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

.file "kernels.S"
; Kernels to measure current consumption
        .text
        .global kernel_fram
        .global kernel_fram_byte
        .global kernel_cache_linear
        .global kernel_cache_sparse
        .global kernel_fram_wsingle
        .global kernel_fram_wsinglebyte
        .global kernel_fram_wsinglesparse
        .global kernel_fram_infloop
        .global LRUTest
        .global TI0HitExample


.text
rpt:
    .word 9000

.text
word_fram_one:
    .word 1
word_fram_two:
    .word 2
word_fram_three:
    .word 3
word_fram_four:
    .word 4
word_fram_five:
    .word 5

kernel_fram:
    mov &rpt, r12
    bis #4, &P1OUT
loop3:
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    mov &word_fram_one, &word_fram_five
    mov &word_fram_two, &word_fram_one
    mov &word_fram_three, &word_fram_five
    mov &word_fram_two, &word_fram_four
    add #-1, r12
    jnz loop3
    bic #4, &P1OUT
    nop
    ret

.text

.align 2
byte_fram_one:
.byte 1
byte_fram_two:
.byte 2
.align 2
byte_fram_three:
.byte 3
.align 2
byte_fram_four:
.byte 4
.align 2
byte_fram_five:
.byte 5
.align 2
kernel_fram_byte:
    mov &rpt, r12
    mov #byte_fram_one, r4
    mov #byte_fram_five, r5

loop4:
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6
    mov.b @r4, r6
    mov.b @r5, r6

    add #-1, r12
    jnz loop4
    nop
    ret

.text
kernel_cache_linear:
    mov &rpt, r12
    mov #0xf00f, r5
    bis #4, &P1OUT
    jmp loop_kcl
.align 8 ; Align to cache-line
loop_kcl:
    swpb r5 ; Cache line 0
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 1
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 2
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 3
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 4
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 0
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 1
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 2
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 3
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 4
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 0
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 1
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 2
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 3
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 4
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 0
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 1
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 2
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 3
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 4
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 0
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 1
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 2
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 3
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 4
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 0
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 1
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 2
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 3
    swpb r5
    swpb r5
    swpb r5
    swpb r5 ; Cache line 4
    swpb r5
    swpb r5
    swpb r5
    add #-1, r12
    jnz loop_kcl
    bic #4, &P1OUT
    nop
    ret

; Jump from one cache line to the next, to see what the miss rate is
; Should be close to 0% miss if next line is fetched on first access of a line
; Should be close to 90% miss otherwise
.text
kernel_cache_sparse:
    mov &rpt, r12
    bis #4, &P1OUT
    jmp loop_kcs
.align 4 ; Align to cache-line
loop_kcs:
    jmp kcs_l0
.align 5
kcs_l0:
    jmp kcs_l1
.align 5
kcs_l1:
    jmp kcs_l2
.align 5
kcs_l2:
    jmp kcs_l3
.align 5
kcs_l3:
    jmp kcs_l4
.align 5
kcs_l4:
    jmp kcs_l5
.align 5
kcs_l5:
    jmp kcs_l6
.align 5
kcs_l6:
    jmp kcs_l7
.align 5
kcs_l7:
    jmp kcs_l8
.align 5
kcs_l8:
    jmp kcs_l9
.align 5
kcs_l9:
    jmp kcs_l10
.align 5
kcs_l10:
    jmp kcs_l11
.align 5
kcs_l11:
    jmp kcs_l12
.align 5
kcs_l12:
    jmp kcs_l13
.align 5
kcs_l13:
    jmp kcs_l14
.align 5
kcs_l14:
    jmp kcs_l15
.align 5
kcs_l15:
    jmp kcs_l16
.align 5
kcs_l16:
    jmp kcs_l17
.align 5
kcs_l17:
    jmp kcs_l18
.align 5
kcs_l18:
    jmp kcs_l19
.align 5
kcs_l19:
    jmp kcs_l20
.align 5
kcs_l20:
    jmp kcs_l21
.align 5
kcs_l21:
    jmp kcs_l22
.align 5
kcs_l22:
    jmp kcs_l23
.align 5
kcs_l23:
    jmp kcs_l24
.align 5
kcs_l24:
    jmp kcs_l25
.align 5
kcs_l25:
    jmp kcs_l26
.align 5
kcs_l26:
    jmp kcs_l27
.align 5
kcs_l27:
    jmp kcs_l28
.align 5
kcs_l28:
    jmp kcs_l29
.align 5
kcs_l29:
    jmp kcs_l30
.align 5
kcs_l30:
    jmp kcs_l31
.align 5
kcs_l31:
    jmp kcs_l32
.align 5
kcs_l32:
    jmp kcs_l33
.align 5
kcs_l33:
    jmp kcs_l34
.align 5
kcs_l34:
    add #-1, r12
    jnz loop_kcs
    bic #4, &P1OUT
    mov &TA0R, r12
    ret

.text
kernel_fram_infloop:

    jmp kernel_fram_infloop

.text
var1:
    .word 0
.align 3
var2:
    .word 0
.align 3
var3:
    .word 0
.align 3
var4:
    .word 0
.align 3
var5:
    .word 0

.data
kernel_fram_wsingle:
    mov &rpt, r12
    mov #1, r5
    bis #4, P1OUT

loop_kfw:
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1
    mov r5, &var1

    add #-1, r12
    jnz loop_kfw
    bic #4, &P1OUT
    ret

.data
kernel_fram_wsinglebyte:
    mov &rpt, r12
    mov #1, r5
    bis #4, P1OUT

loop_kfwb:
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1
    mov.b r5, &var1

    add #-1, r12
    jnz loop_kfwb
    bic #4, &P1OUT
    ret

.data
kernel_fram_wsinglesparse:
    mov &rpt, r12
    mov #1, r5
    bis #4, P1OUT

loop_kfwss:
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    mov r5, &var1
    mov r5, &var2
    mov r5, &var3
    mov r5, &var4
    mov r5, &var5
    add #-1, r12
    jnz loop_kfwss
    bic #4, &P1OUT
    ret

; Jump 8 words every time (2 bytes for jmp instruction followed by 6 nops)
; Forces all accesses to go to the same cache line
; First miss: Label_A gets filled into the cache line
; Second miss: Label_B gets filled into the cache line
; Third miss: Label_C fills the cache line (and evicts Label_A)
; Fourth miss: Label_A fills the cache line (and evicts Label_B)
; Fifth miss: Label_B fills the cache line (and evicts Label_C)
.text
.align 8
TI0HitExample:
    nop
    nop
    nop
Label_A:
    jmp Label_B
    nop
    nop
    nop
    nop
    nop
    nop
Label_B:
    jmp Label_C
    nop
    nop
    nop
    nop
    nop
    nop
Label_C:
    jmp Label_A
    nop
    nop
    nop

; Test whether the cache replacement policy is LRU or FIFO/round robin
; Expect 50% miss rate for LRU, or 75% for FIFO
; Label A and C are in the same cache line
; Label B, D are mapped to the same set, but with different tags
; 1 Miss: Label_A gets fetched
; 2 Miss: Label_B gets fetched
; 3 Hit: Label_C should already be present
; 4 Miss: Label_D gets fetched. (A&C) or B needs to be evicted. LRU evicts B, FIFO evicts (A&C)
; STEADY STATE:
; 5 Hit: Label_A should already be present
; 6 Miss: Label_B gets fetched
; 7 Hit: Label_C should already be present
; 4 Miss: Label_D gets fetched. (A&C) or B needs to be evicted. LRU evicts B, FIFO evicts (A&C)
LRUTest:
    bis #4, &P1OUT
    mov #0xFFFF, r12
    mov #-1, r13
    jmp LRU_Label_A

.text
.align 5
LRU_Label_A:
    jmp LRU_Label_B
LRU_Label_C:
    jmp LRU_Label_D
    nop
    nop
    nop
    nop
    nop
    nop
    nop
.align 7
LRU_Label_B:
    jmp LRU_Label_C
    nop
    nop
    nop
    nop
    nop
    nop
    nop
.align 7
LRU_Label_D:
    add r13, r12
    jnz LRU_Label_A
    nop
    bic #4, &P1OUT
    ret
    nop
    nop
    nop
    nop
