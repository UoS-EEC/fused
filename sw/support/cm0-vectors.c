/******************************************************************************
 * @file     startup_ARMCM0.c
 * @brief    CMSIS-Core(M) Device Startup File for a Cortex-M0 Device
 * @version  V2.0.2
 * @date     15. November 2019
 ******************************************************************************/
/*
 * Copyright (c) 2009-2019 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Modified 2020
 * Copyright (c) 2020 University of Southampton.
 */

#include <fused.h>
#include <stdint.h>
#include "cm0.h"
#include "support.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __stack_high;
extern __NO_RETURN void _start(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
void Default_Handler(void);
void Reset_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
extern void NMI_Handler(void);
extern void HardFault_Handler(void);
extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

extern void Interrupt0_Handler(void);
extern void Interrupt1_Handler(void);
extern void Interrupt2_Handler(void);
extern void Interrupt3_Handler(void);
extern void Interrupt4_Handler(void);
extern void Interrupt5_Handler(void);
extern void Interrupt6_Handler(void);
extern void Interrupt7_Handler(void);
extern void Interrupt8_Handler(void);
extern void Interrupt9_Handler(void);
extern void Interrupt10_Handler(void);
extern void Interrupt11_Handler(void);
extern void Interrupt12_Handler(void);
extern void Interrupt13_Handler(void);
extern void Interrupt14_Handler(void);
extern void Interrupt15_Handler(void);
extern void Interrupt16_Handler(void);
extern void Interrupt17_Handler(void);
extern void Interrupt18_Handler(void);
extern void Interrupt19_Handler(void);
extern void Interrupt20_Handler(void);
extern void Interrupt21_Handler(void);
extern void Interrupt22_Handler(void);
extern void Interrupt23_Handler(void);
extern void Interrupt24_Handler(void);
extern void Interrupt25_Handler(void);
extern void Interrupt26_Handler(void);
extern void Interrupt27_Handler(void);
extern void Interrupt28_Handler(void);
extern void Interrupt29_Handler(void);
extern void Interrupt30_Handler(void);
extern void Interrupt31_Handler(void);

/* Exceptions */
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));

void Interrupt0_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt1_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt2_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt3_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt4_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt5_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt6_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt7_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt8_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt9_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt10_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt11_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt12_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt13_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt14_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt15_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt16_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt17_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt18_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt19_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt20_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt21_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt22_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt23_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt24_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt25_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt26_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt27_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt28_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt29_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt30_Handler(void) __attribute__((weak, alias("Default_Handler")));
void Interrupt31_Handler(void) __attribute__((weak, alias("Default_Handler")));

/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/

//#if defined ( __GNUC__ )
//#pragma GCC diagnostic push
//#pragma GCC diagnostic ignored "-Wpedantic"
//#endif

extern VECTOR_TABLE_Type __VECTOR_TABLE[48];
VECTOR_TABLE_Type __VECTOR_TABLE[48] __VECTOR_TABLE_ATTRIBUTE = {
    (VECTOR_TABLE_Type)(&__stack_high), /*     Initial Stack Pointer */
    Reset_Handler,                      /*     Reset Handler */
    NMI_Handler,                        /* -14 NMI Handler */
    HardFault_Handler,                  /* -13 Hard Fault Handler */
    0,                                  /*     Reserved */
    0,                                  /*     Reserved */
    0,                                  /*     Reserved */
    0,                                  /*     Reserved */
    0,                                  /*     Reserved */
    0,                                  /*     Reserved */
    0,                                  /*     Reserved */
    SVC_Handler,                        /*  -5 SVCall Handler */
    0,                                  /*     Reserved */
    0,                                  /*     Reserved */
    PendSV_Handler,                     /*  -2 PendSV Handler */
    SysTick_Handler,                    /*  -1 SysTick Handler */

    /* Interrupts */
    Interrupt0_Handler,  /*   0 Interrupt 0 */
    Interrupt1_Handler,  /*   1 Interrupt 1 */
    Interrupt2_Handler,  /*   2 Interrupt 2 */
    Interrupt3_Handler,  /*   3 Interrupt 3 */
    Interrupt4_Handler,  /*   4 Interrupt 4 */
    Interrupt5_Handler,  /*   5 Interrupt 5 */
    Interrupt6_Handler,  /*   6 Interrupt 6 */
    Interrupt7_Handler,  /*   7 Interrupt 7 */
    Interrupt8_Handler,  /*   8 Interrupt 8 */
    Interrupt9_Handler,  /*   9 Interrupt 9 */
    Interrupt10_Handler, /*   10 Interrupt 10 */
    Interrupt11_Handler, /*   11 Interrupt 11 */
    Interrupt12_Handler, /*   12 Interrupt 12 */
    Interrupt13_Handler, /*   13 Interrupt 13 */
    Interrupt14_Handler, /*   14 Interrupt 14 */
    Interrupt15_Handler, /*   15 Interrupt 15 */
    Interrupt16_Handler, /*   16 Interrupt 16 */
    Interrupt17_Handler, /*   17 Interrupt 17 */
    Interrupt18_Handler, /*   18 Interrupt 18 */
    Interrupt19_Handler, /*   19 Interrupt 19 */
    Interrupt20_Handler, /*   20 Interrupt 20 */
    Interrupt21_Handler, /*   21 Interrupt 21 */
    Interrupt22_Handler, /*   22 Interrupt 22 */
    Interrupt23_Handler, /*   23 Interrupt 23 */
    Interrupt24_Handler, /*   24 Interrupt 24 */
    Interrupt25_Handler, /*   25 Interrupt 25 */
    Interrupt26_Handler, /*   26 Interrupt 26 */
    Interrupt27_Handler, /*   27 Interrupt 27 */
    Interrupt28_Handler, /*   28 Interrupt 28 */
    Interrupt29_Handler, /*   29 Interrupt 29 */
    Interrupt30_Handler, /*   30 Interrupt 30 */
    Interrupt31_Handler  /*   31 Interrupt 31 */
};

//#if defined ( __GNUC__ )
//#pragma GCC diagnostic pop
//#endif

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void) {
  // SystemInit();                             /* CMSIS System Initialization */
  //__PROGRAM_START();                        /* Enter PreMain (C library entry
  // point) */
  _start();
}

/*----------------------------------------------------------------------------
  Hard Fault Handler
 *----------------------------------------------------------------------------*/
void HardFault_Handler(void) {
  SIMPLE_MONITOR = SIMPLE_MONITOR_SW_ERROR;
  while (1)
    ;
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) {
  SIMPLE_MONITOR = SIMPLE_MONITOR_SW_ERROR;
  while (1)
    ;
}
