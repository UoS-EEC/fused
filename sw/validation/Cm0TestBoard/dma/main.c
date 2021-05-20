/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <support.h>

#include "cmsis/core_cm0.h"
#include "dma_defs.h"

/* ------ Types -------------------------------------------------------------*/

typedef struct {
  volatile unsigned DMACTL0;
  volatile unsigned DMACTL1;
  volatile unsigned DMACTL2;
  volatile unsigned DMACTL4;

  volatile unsigned DMA0CTL;
  volatile unsigned DMA0SA;
  volatile unsigned DMA0DA;
  volatile unsigned DMA0SZ;

  volatile unsigned DMA1CTL;
  volatile unsigned DMA1SA;
  volatile unsigned DMA1DA;
  volatile unsigned DMA1SZ;

  volatile unsigned DMA2CTL;
  volatile unsigned DMA2SA;
  volatile unsigned DMA2DA;
  volatile unsigned DMA2SZ;

  volatile unsigned DMA3CTL;
  volatile unsigned DMA3SA;
  volatile unsigned DMA3DA;
  volatile unsigned DMA3SZ;

  volatile unsigned DMA4CTL;
  volatile unsigned DMA4SA;
  volatile unsigned DMA4DA;
  volatile unsigned DMA4SZ;

  volatile unsigned DMA5CTL;
  volatile unsigned DMA5SA;
  volatile unsigned DMA5DA;
  volatile unsigned DMA5SZ;

  volatile unsigned DMAIV;

} Dma_t;

volatile Dma_t *Dma = (Dma_t *)DMA_BASE;

/* ------ Extern functions --------------------------------------------------*/

/* ------ Function Prototypes -----------------------------------------------*/
void assert(bool c);

/* ------ Variable Declarations ---------------------------------------------*/
volatile static bool irqTriggered = false;

/* ------ Function Declarations ---------------------------------------------*/

void Interrupt27_Handler(void) { // DMA interrupt handler
  irqTriggered = true;
  Dma->DMAIV = 0; // Clear pending irq
}

void dma_copy(char *src, char *dst, const int len) {
  // Blocking block-transfer, autoincrement, 4-byte word size
  Dma->DMA0CTL = DMADT_1 | DMASRCINCR_3 | DMADSTINCR_3;
  Dma->DMA0SA = (unsigned)src;
  Dma->DMA0DA = (unsigned)dst;
  Dma->DMA0SZ = len / 4;
  Dma->DMA0CTL |= DMAEN | DMAREQ;

  // Transfer tail bytes
  for (int i = len % 4; i > 0; --i) {
    *(dst + len - i) = *(src + len - i);
  }
}

static int a[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static int b[sizeof(a)] = {0};

int main(void) {
  while (1) {
    // Block copy
    dma_copy((char *)a, (char *)b, sizeof(a));

    // Wait for transfer to complete
    while (Dma->DMA0CTL & DMAEN)
      ;

    // Verify results
    for (size_t i = 0; i < sizeof(a) / sizeof(int); i++) {
      assert(b[i] == a[i]);
    }

    // Block copy with interrupt
    Dma->DMA0CTL = DMADT_1 | DMASRCINCR_3 | DMADSTINCR_3 | DMAIE__ENABLE;
    Dma->DMA0SA = (unsigned)a;
    Dma->DMA0DA = (unsigned)b;
    Dma->DMA0SZ = sizeof(a) / 4;

    NVIC_SetPriority(27, 2);
    NVIC_EnableIRQ(27);

    Dma->DMA0CTL |= DMAEN | DMAREQ;

    // Delay while transfering
    for (int i = 0; i < 100; ++i) {
    }

    // Assert that interrupt was triggered
    assert(irqTriggered);

    end_experiment();
  }
}

void assert(bool c) {
  if (!c) {
    indicate_test_fail();
    while (1)
      ; // stall
  }
}
