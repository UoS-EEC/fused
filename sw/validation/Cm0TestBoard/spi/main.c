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
#include "stm32f0_spi.h"

/* ------ Macros ------------------------------------------------------------*/

/* ------ Extern functions --------------------------------------------------*/

/* ------ Function Prototypes -----------------------------------------------*/

/* ------ Variable Declarations ---------------------------------------------*/

const uint8_t txpayload[] = "Hello World!";
static uint8_t rxpayload[sizeof(txpayload) + 1];
volatile static int rxIdx = 0;

/* ------ Function Declarations ---------------------------------------------*/

volatile static int nTicks = 0;
volatile static SPI_TypeDef *spi = ((SPI_TypeDef *)SPI1_BASE);

void Interrupt25_Handler(void) {  // SPI interrupt handler
  while (spi->SR & SPI_SR_FRLVL) {
    rxpayload[rxIdx++] = *((uint8_t *)&spi->DR);
  }
}

/**
 * @brief test basic functionality of the SPI
 *      - Set up an interrupt
 *      - Test passes after 10 interrupts
 */
void main(void) {
  indicate_workload_begin();
  spi->CR2 = (7u << SPI_CR2_DS_Pos) |   // 8 bit data size
             SPI_CR2_FRXTH |            // 1-byte RXNE threshold
             SPI_CR2_RXNEIE;            // RX interrupt
  spi->CR1 = (1u << SPI_CR1_BR_Pos) |   // Baudrate = clk/4
             (1u << SPI_CR1_SPE_Pos) |  // SPI enable
             (1u << SPI_CR1_MSTR_Pos);  // Master mode
  NVIC_SetPriority(25, 2);
  NVIC_EnableIRQ(25);

  for (int i = 0; i < sizeof(txpayload); ++i) {
    *((uint8_t *)&spi->DR) = txpayload[i];
    while (!(spi->SR & SPI_SR_TXE))
      ;  // Wait for a free tx slot
  }

  *((uint8_t *)&spi->DR) = 0xAA;  // Garbage last byte (rx lags by one)

  while (rxIdx < sizeof(rxpayload)) {
    // Wait for transmission to complete
  }

  // Check equality
  for (int i = 0; i < sizeof(txpayload); i++) {
    if (rxpayload[i] != txpayload[i]) {
      indicate_test_fail();
    }
  }
  end_experiment();
}
