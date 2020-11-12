/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <math.h>
#include <msp430fr5994.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <support.h>

/* ------ Macros ------------------------------------------------------------*/

/* ------ Extern functions --------------------------------------------------*/

/* ------ Function Prototypes -----------------------------------------------*/
void assert(bool c);

/* ------ Variable Declarations ---------------------------------------------*/
static bool irqTriggered = false;

/* ------ Function Declarations ---------------------------------------------*/

void __attribute__((__interrupt__(DMA_VECTOR), optimize(1))) dma_isr(void) {
  irqTriggered = true;
  DMAIV = 0;  // Clear pending irq
}

void dma_copy(char* src, char* dst, const int len) {
  // Blocking block-transfer, autoincrement, 2-byte word size
  DMA0CTL = DMADT_1 | DMASRCINCR_3 | DMADSTINCR_3;
  DMA0SA = (uint16_t)src;
  DMA0DA = (uint16_t)dst;
  DMA0SZ = (uint16_t)(len / 2);
  DMA0CTL |= DMAEN | DMAREQ;
  while (DMA0CTL & DMAEN)
    ;             // Wait for transfer complete
  if (len % 2) {  // transfer last byte
    *(dst + len - 1) = *(src + len - 1);
  }
}

void dma_triggered(char* src, char* dst) {
  // Single-transfer, autoincrement destination, 2-byte word size, enable
  // interrupt
  DMA0CTL = DMADT_0 | DMASRCINCR_0 | DMADSTINCR_3 | DMAIE;
  DMACTL0 = DMA0TSEL__DMA0TRIG1;  // Timer TA0
  DMA0SA = (uint16_t)src;
  DMA0DA = (uint16_t)dst;
  DMA0SZ = 5;
  DMA0CTL |= DMAEN;

  // Up-mode, aclk, count to 100, no interrupts
  TA0CTL = MC_0 | TACLR;  // Disable
  TA0CCR0 = 100;
  TA0CTL = MC_1 | TASSEL_1;

  while (DMA0CTL & DMAEN)
    ;  // Wait for transfer complete
  TA0CTL = MC_0 | TACLR;
}

static int a[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static int b[sizeof(a)] = {0};

int main(void) {
  while (1) {
    // Block copy
    dma_copy((char*)a, (char*)b, sizeof(a));
    for (size_t i = 0; i < sizeof(a) / sizeof(int); i++) {
      assert(b[i] == a[i]);
    }

    // Triggered capture, using timer
    int c = 0xAAAA;
    __enable_interrupt();
    dma_triggered((char*)&c, (char*)b);
    for (size_t i = 0; i < 5; i++) {
      assert(b[i] == 0xAAAA);
    }
    assert(irqTriggered);

    end_experiment();
  }
}

void assert(bool c) {
  if (!c) {
    indicate_test_fail();
    while (1)
      ;  // stall
  }
}
