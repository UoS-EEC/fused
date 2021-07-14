/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <support.h>

#include "accelerometer.h"
#include "stm32f0_spi.h"
#include <stdbool.h>
#include <stdint.h>

#define FCLK 8000000
#define PIN_ACCELEROMETER_CS (1u << 17)
#define PIN_ACCELEROMETER_IRQ (1u << 18)

/* ------ Function Prototypes -----------------------------------------------*/

// Initialize GPIO settings
static void gpioInit();

// Transmit data
void spiTransmit(const uint8_t *data, int len);

// Transmit data, and get response
uint8_t spiTransaction(const uint8_t data);

// Send a command to the accelerometer
static void accelerometerCommand(const uint8_t address, const uint8_t data);

// Read some data from the accelerometer into data
static void accelerometerRead(const uint8_t address, uint8_t *data, int len);

// Assert c==true, otherwise indicate test fail and stall.
void assert(bool c);

/* ------ Global Variables --------------------------------------------------*/
volatile static SPI_TypeDef *spi = ((SPI_TypeDef *)SPI1_BASE);

struct {
  uint8_t data[100];
  volatile int len;
} spiPacket;

/* ------ Function Definitions ----------------------------------------------*/

__attribute__((optimize(1))) static inline void __delay_cycles(volatile int n) {
  while (--n > 0) {
    __asm__ volatile("nop" :);
  }
}

static inline void assert_accelerometer_cs() {
  Gpio->DATA.WORD &= ~PIN_ACCELEROMETER_CS;
}

static inline void deassert_accelerometer_cs() {
  Gpio->DATA.WORD |= PIN_ACCELEROMETER_CS;
}

void Interrupt25_Handler(void) { // SPI interrupt handler
  while (spi->SR & SPI_SR_FRLVL) {
    spiPacket.data[spiPacket.len++] = *((uint8_t *)&spi->DR);
  }
}

void Interrupt0_Handler() { // GPIO interrupt handler
}

uint8_t spiTransaction(const uint8_t data) {
  while (!(spi->SR & SPI_SR_TXE))
    ; // Wait for a free tx slot
  *((uint8_t *)&spi->DR) = data;
  while (!(spi->SR & SPI_SR_RXNE))
    ; // Wait for transaction complete
  return *((uint8_t *)&spi->DR);
}

static void accelerometerCommand(const uint8_t address, const uint8_t data) {
  assert(!(address & ACC_R_REG));
  assert_accelerometer_cs();
  spiTransaction(address | ACC_W_REG);
  spiTransaction(data);
  deassert_accelerometer_cs();
}

static void accelerometerRead(const uint8_t address, uint8_t *data, int len) {
  assert(!(address & ACC_W_REG));
  assert_accelerometer_cs();
  spiTransaction(address | ACC_R_REG); // Set up address
  for (int i = 0; i < len; ++i) {
    data[i] = spiTransaction(0); // Send dummy data while reading
  }
  deassert_accelerometer_cs();
}

int main(void) {
  indicate_workload_begin();
  target_init();

  /* ------ Setup I/O ------ */
  gpioInit();

  /* ------ Setup SPI ------ */
  // Initialize SPI as Master
  spi->CR2 = (7u << SPI_CR2_DS_Pos) |  // 8 bit data size
             SPI_CR2_FRXTH;            // 1-byte RXNE threshold
  spi->CR1 = (2u << SPI_CR1_BR_Pos) |  // Baudrate = clk/8 = 1MHz
             (1u << SPI_CR1_SPE_Pos) | // SPI enable
             (1u << SPI_CR1_MSTR_Pos); // Master mode

  uint8_t buf[512];

  /* ------ Sleep -> Standby ------ */
  accelerometerCommand(ACC_CTRL, ACC_CTRL_MODE_STANDBY);

  // Wait ~1ms
  __delay_cycles(8000);

  // Status should indicate not busy
  accelerometerRead(ACC_STATUS, buf, 1);
  assert(!(*buf & ACC_STATUS_BUSY));

  /* ------ Single-shot measurement ------ */
  accelerometerCommand(ACC_CTRL_FS, 9); // (10KHz/(9+1) => 1 ms sampling time
  // Start sampling all axes
  accelerometerCommand(ACC_CTRL, ACC_CTRL_X_EN | ACC_CTRL_Y_EN | ACC_CTRL_Z_EN |
                                     ACC_CTRL_MODE_SINGLE | ACC_CTRL_IE);
  for (volatile int timeout = 1000; timeout >= 0; --timeout) {
    if (Gpio->DATA.WORD & PIN_ACCELEROMETER_IRQ) {
      break;
    }
    if (timeout == 0) {
      indicate_test_fail(); // timeout
    }
  }

  // Read out a frame
  accelerometerRead(ACC_DATA, buf, 4);
  assert(buf[0] == 7);
  assert(buf[1] == 0);
  assert(buf[2] == 0);
  assert(buf[3] == 62);

  // Status should indicate not busy after reading out frame
  accelerometerRead(ACC_STATUS, buf, 1);
  assert(!(*buf & ACC_STATUS_BUSY));

  /* ------ Continuous measurement ------ */
  accelerometerCommand(ACC_CTRL_FS, 0);    // 0.1 ms sampling time
  accelerometerCommand(ACC_FIFO_THR, 128); // 32 samples

  // Start sampling all axes
  accelerometerCommand(ACC_CTRL, ACC_CTRL_X_EN | ACC_CTRL_Y_EN | ACC_CTRL_Z_EN |
                                     ACC_CTRL_MODE_CONTINUOUS | ACC_CTRL_IE);
  for (volatile int timeout = 10000; timeout >= 0; --timeout) {
    if (Gpio->DATA.WORD & PIN_ACCELEROMETER_IRQ) {
      break;
    }
    if (timeout == 0) {
      indicate_test_fail(); // timeout
    }
  }

  // Go to standby mode
  accelerometerCommand(ACC_CTRL, ACC_CTRL_MODE_STANDBY);

  // Read out a frame
  accelerometerRead(ACC_DATA, buf, 4);
  assert(buf[0] == 7);
  assert(buf[1] == 0);
  assert(buf[2] == 0);
  assert(buf[3] == 62);

  // Read out a lot more frames
  accelerometerRead(ACC_DATA, buf, 124);

  // Status should indicate not busy after reading out frames
  accelerometerRead(ACC_STATUS, buf, 1);
  assert(!(*buf & ACC_STATUS_BUSY));

  end_experiment();
  return 0;
}

void assert(bool c) {
  if (!c) {
    indicate_test_fail();
    while (1)
      ; // Stall
  }
}

static void gpioInit() {
  Gpio->DATA.WORD = 0;
  Gpio->DIR.WORD = PIN_ACCELEROMETER_CS;
}
