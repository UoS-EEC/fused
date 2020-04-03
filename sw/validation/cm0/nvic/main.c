/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <support.h>
#include "cmsis/core_cm0.h"

/* ------ Macros ------------------------------------------------------------*/

/* ------ Extern functions --------------------------------------------------*/

/* ------ Function Prototypes -----------------------------------------------*/

/* ------ Variable Declarations ---------------------------------------------*/

/* ------ Function Declarations ---------------------------------------------*/

volatile static int nTicks = 0;

void Interrupt0_Handler() {
  if (nTicks == 10) {
    end_experiment();  // Test passed
  } else if (nTicks > 10) {
    // Something weird must've happened -- Test failed
    indicate_test_fail();
  }
  nTicks++;
  NVIC_ClearPendingIRQ(0);
}

/**
 * @brief test basic functionality:
 *      - Set up an interrupt
 *      - Test passes after 10 interrupts
 */
void main(void) {
  NVIC_SetPriority(0, 2);
  NVIC_EnableIRQ(0);

  while (1) {
    NVIC_SetPendingIRQ(0);
    for (unsigned i = 0; i < 10000; i++) {
      // Wait a while
    }
    if (nTicks == 0) {
      indicate_test_fail();  // Should have seen a tick by now
    }
  }
}
