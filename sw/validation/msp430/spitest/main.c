/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <msp430fr5994.h>
#include <stdbool.h>
#include <stdint.h>
#include <support.h>
#include "eusci_b_spi.h"

/* ------ Macros ------------------------------------------------------------*/

/* ------ Extern functions --------------------------------------------------*/

/* ------ Function Prototypes -----------------------------------------------*/

/* ------ Variable Declarations ---------------------------------------------*/
uint8_t TXData = 0x01; 
uint8_t RXData = 0x00;

/* ------ Function Declarations ---------------------------------------------*/

int main(void) {
  WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer


  // Initialize Master
  EUSCI_B_SPI_initMasterParam param = {0};
  param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
  param.clockSourceFrequency = 1000000;  
  param.desiredSpiClock = 500000;
  param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
  param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CHANGED_ONFIRST_CAPTURED_ON_NEXT;
  param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_HIGH;
  param.spiMode = EUSCI_B_SPI_3PIN;
  EUSCI_B_SPI_initMaster(EUSCI_B0_BASE, &param);

  // Enable SPI module
  EUSCI_B_SPI_enable(EUSCI_B0_BASE);

  EUSCI_B_SPI_clearInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);
  // Enable USCI_B0 RX interrupt
  EUSCI_B_SPI_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_SPI_RECEIVE_INTERRUPT);

  __enable_interrupt();

  // USCI_B0 TX buffer ready?
  while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                                         EUSCI_B_SPI_TRANSMIT_INTERRUPT));

  // Transmit Data to slave
  EUSCI_B_SPI_transmitData(EUSCI_B0_BASE, TXData);

  for(;;);
  
}

__attribute__((interrupt(USCI_B0_VECTOR)))
void USCI_B0_ISR (void)
{
    switch (__even_in_range(UCB0IV, USCI_SPI_UCTXIFG))
    {
        case USCI_SPI_UCRXIFG:      // UCRXIFG
            //USCI_B0 TX buffer ready?
            while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B0_BASE,
                        EUSCI_B_SPI_TRANSMIT_INTERRUPT));

            RXData = EUSCI_B_SPI_receiveData(EUSCI_B0_BASE);

            //Increment data
            TXData++;

            //Send next value
            EUSCI_B_SPI_transmitData(EUSCI_B0_BASE,
                TXData
                );

            //Delay between transmissions for slave to process information
            __delay_cycles(40);
            break;
        default:
            break;
    }
}

