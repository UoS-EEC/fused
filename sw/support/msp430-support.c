/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include "msp430-support.h"

#if TEXT_IN_SRAM && DATA_IN_SRAM
#define FRAM_WAIT_MULT 4
#elif TEXT_IN_SRAM
#define FRAM_WAIT_MULT (4 - FRAM_WAIT / 8)
#elif DATA_IN_SRAM
#define FRAM_WAIT_MULT (4 - FRAM_WAIT / 8)
#else
#define FRAM_WAIT_MULT (4 - FRAM_WAIT / 8)
#endif

static void gpio_init();
static void cs_init();
static void __stack_set();

void indicate_workload_begin() {
  P1OUT |= BIT2;
#ifdef SIMULATION
  SIMPLE_MONITOR = SIMPLE_MONITOR_INDICATE_BEGIN;
#endif
}

void indicate_workload_end() {
  P1OUT &= ~BIT2;
#ifdef SIMULATION
  SIMPLE_MONITOR = SIMPLE_MONITOR_INDICATE_END;
#endif
}

void indicate_test_fail() {
  P1OUT &= ~BIT3;
#ifdef SIMULATION
  SIMPLE_MONITOR = SIMPLE_MONITOR_TEST_FAIL;
#endif
  while (1)
    ;
}

void end_experiment() {
  P1OUT &= ~BIT4;
#ifdef SIMULATION
  SIMPLE_MONITOR = SIMPLE_MONITOR_KILL_SIM;
#endif
  while (1)
    ;
}

void wait() {
  for (volatile uint32_t i = 0; i < FRAM_WAIT_MULT * 10000ll; i++)
    ;  // delay
}

void target_init() {
  cs_init();
  gpio_init();
}

static void cs_init() {
  CSCTL0_H = 0xA5;            // Unlock CS
  FRCTL0_H = 0xA5;            // Unlock FRAM ctrl
  FRCTL0_L = FRAM_WAIT << 4;  // wait states

  CSCTL1 = DCORSEL | DCOFSEL_4;  // 16 MHz

  // Set ACLK = VLO; MCLK = DCO; SMCLK = DCO
  CSCTL2 = SELA_1 + SELS_3 + SELM_3;

  // ACLK, SMCLK, MCLK dividers
  CSCTL3 = DIVA_0 + DIVS_1 + DIVM_1;
}

static void gpio_init() {
  // Need to initialize all ports/pins to reduce power consumption
  P1OUT = 0;  // LEDs on P1.0 and P1.1
  P1DIR = 0xff;
  P2OUT = 0;
  P2DIR = 0xff;
  P3OUT = 0;
  P3DIR = 0xff;
  P4OUT = BIT0;  // Pull-up on board
  P4DIR = 0xff;
  P6OUT = 0;
  P6DIR = 0xff;
  P7OUT = 0;
  P7DIR = 0xff;
  P8OUT = 0;
  P8DIR = 0xff;
  PM5CTL0 &= ~LOCKLPM5;
}

// Reset stack to 0s during boot (for clean backtrace when debugging)
__attribute__((optimize(0))) static void __stack_set() {
  extern uint8_t __stack_low, __stack_high;
  register uint16_t *start = (uint16_t *)&__stack_low;
  register uint16_t *end = (uint16_t *)&__stack_high;
  while (start < end) {
    *start++ = 0x00;
  }
}

// Boot
__attribute__((interrupt(RESET_VECTOR), naked, used, optimize(0),
               section(".ftext"))) void
_start() {
  extern uint8_t __stack_low, __stack_high;
  WDTCTL = (WDTPW | WDTHOLD);
  __set_SP_register(&__stack_high);  // Set SP
  __bic_SR_register(GIE);            // Disable interrupts during boot

  register uint16_t *dst __attribute__((unused));
  register uint16_t *src __attribute__((unused));
  register uint16_t *end __attribute__((unused));

  // Load constants
  extern const uint8_t __rodata_low, __rodata_high, __rodata_loadLow;
  dst = (uint16_t *)&__rodata_low;
  src = (uint16_t *)&__rodata_loadLow;
  end = (uint16_t *)&__rodata_high;
  while ((dst <= end) && (dst != src)) {
    *dst++ = *src++;
  }

  // Load code
  extern const uint8_t __text_low, __text_high, __text_loadLow;
  /* Don't use memcpy here --it isn't loaded yet! */
  dst = (uint16_t *)&__text_low;
  src = (uint16_t *)&__text_loadLow;
  end = (uint16_t *)&__text_high;
  while ((dst <= end) && (dst != src)) {
    *dst++ = *src++;
  }

  // Load data
  extern const uint8_t __data_low, __data_high, __data_loadLow;
  dst = (uint16_t *)&__data_low;
  src = (uint16_t *)&__data_loadLow;
  end = (uint16_t *)&__data_high;
  while ((dst <= end) && (dst != src)) {
    *dst++ = *src++;
  }

  int main();
  main();
}

// Flush cache
__attribute__((optimize(0))) void cache_flush() {
  // Fill cache with garbage
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
  __asm__(" nop \r\n");
}
