#
# Copyright (c) 2019-2020, University of Southampton and Contributors.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0
#

set pagination off
set verbose off

#layout asm
#layout regs
#

#file
#b __stack_set
b main

target remote :55000
load
c

set $r4 = 0
set $r5 = 0
set $r6 = 0
set $r7 = 0
set $r8 = 0
set $r9 = 0
set $r10 = 0
set $r11 = 0
set $r12 = 0
set $r13 = 0
set $r14 = 0
set $r15 = 0
info registers


set $i = 0

while ($i < 10000)
  si
  info registers
  set $i = $i + 1
  end

  k
  q
