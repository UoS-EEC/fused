/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

EXTERN (_vectors)
EXTERN (__VECTOR_TRABLE)
ENTRY(Reset_Handler)

__stack_size = 0x200;

MEMORY
{
        /*rom (rwx) : ORIGIN = 0x08000000, LENGTH = 16K */
        /*ram (rwx) : ORIGIN = 0x20000000, LENGTH = 4K */
        /*invm (rx) : ORIGIN = 0x20000000, LENGTH = 8K */
        rom (rx)   : ORIGIN = 0x08000000, LENGTH = 64K
        nvram (rwx): ORIGIN = 0x20000000, LENGTH = 16K
        sram (rwx) : ORIGIN = 0x20000000 + 16K, LENGTH = 8K
}

/* Define sections. */
SECTIONS
{

  .vectors : {
    . = ALIGN(4);
    KEEP(*(.vectors)) /* Vector table */
  } >rom

  .text : {
    . = ALIGN(4);
    *(.text*)   /* Program code */
    *(.text.*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    .etext = .;
  } >rom

  .rodata : {
    . = ALIGN(4);
    *(.rodata*)  /* Read-only data */
    *(.rodata.*) /* Read-only data */
    . = ALIGN(4);
  } >rom

  .ftext : {
    . = ALIGN(4);
    *(.ftext)
    . = ALIGN(4);
  } >rom

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >rom
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >rom

  .preinit_array : {
    __preinit_array_start = .;
    KEEP (*(.preinit_array*))
    __preinit_array_end = .;
  } >rom
  .init_array : {
    __init_array_start = .;
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    __init_array_end = .;
  } >rom
  .fini_array : {
    __fini_array_start = .;
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    __fini_array_end = .;
  } >rom

  .data : {
    _data = .;
    _sidata = .;
    PROVIDE(__data_low = .);
    . = ALIGN(4);
    *(.data*)   /* Read-write initialized data */
    . = ALIGN(4);
    _edata = .;
    PROVIDE(__data_high = .);
  } >nvram

  _data_loadaddr = LOADADDR(.data);
  PROVIDE(__data_loadLow = LOADADDR(.data));
  PROVIDE(__data_loadHigh = LOADADDR(.data) + SIZEOF(.data));

  .bss (NOLOAD) : {
    . = ALIGN(4);
     __bss_start__ = .;
    PROVIDE(__bss_low = .);
    *(.bss)
    *(.bss*)    /* Read-write zero initialized data */
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = .;
    PROVIDE(__bss_end = .);
  } >nvram

.heap (NOLOAD) :
{
  . = ALIGN(2);
  __heap_start__ = .;
  KEEP (*(.heap))
  _end = .;
  PROVIDE (end = .);
  /* This word is here so that the section is not empty, and thus
     not discarded by the linker.  The actual value does not matter
     and is ignored.  */
  LONG(0);
  __heap_end__ = .;
  __HeapLimit = __heap_end__;
} > nvram

.stack (ORIGIN (nvram) + LENGTH(nvram) - __stack_size) :
{
  . = ALIGN(4);
  PROVIDE (__stack = .);
  __stack_low = .;
  *(.stack)
  . += __stack_size;
  __stack_high = .;
  PROVIDE (__STACK_END = .);
}

  PROVIDE(_stack = ORIGIN(nvram) + LENGTH(nvram));
  PROVIDE(_stacktop = ORIGIN(nvram) + LENGTH(nvram));
  PROVIDE(heapCurrent = ORIGIN(nvram));
  . = ALIGN(4);
  end = .;

  /* Remove information from standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes)}
}
