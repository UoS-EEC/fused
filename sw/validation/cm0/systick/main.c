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

void SysTick_Handler(void) {  // SysTick Interrupt Handler
  if (nTicks == 10) {
    end_experiment();  // Test passed
  } else if (nTicks > 10) {
    // Something weird must've happened -- Test failed
    indicate_test_fail();
  }

  nTicks++;
}

/**
 * @brief test basic functionality:
 *      - Set up an interrupt
 *      - Test passes after 10 interrupts
 */
void main(void) {
  // Set an interrupt every 1 ms
  if (SysTick_Config((SysTick->CALIB & SysTick_CALIB_TENMS_Msk) / 10)) {
    indicate_test_fail();  // Error
  }
  while (1) {
    for (unsigned i = 0; i < 10000; i++) {
      // Wait a while
    }
    if (nTicks == 0) {
      indicate_test_fail();  // Should have seen a tick by now
    }
  }
}
