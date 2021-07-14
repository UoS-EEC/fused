/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <support.h>

#include "bme280.h"
#include "stm32f0_spi.h"
#include <stdbool.h>
#include <stdint.h>

#define FCLK 8000000
#define PIN_BME280_CS (1u << 16)

/* ------ Function Prototypes -----------------------------------------------*/

// Initialize GPIO settings
static void gpioInit();

// Transmit data
void spiTransmit(const uint8_t *data, int len);

// Transmit data, and get response
uint8_t spiTransaction(const uint8_t data);

// Send a command to the bme280
static void bme280Command(const uint8_t address, const uint8_t data);

// Read some data from the bme280 into data
static void bme280Read(const uint8_t address, uint8_t *data, int len);

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

static inline void assert_bme280_cs() { Gpio->DATA.WORD &= ~PIN_BME280_CS; }

static inline void deassert_bme280_cs() { Gpio->DATA.WORD |= PIN_BME280_CS; }

void Interrupt25_Handler(void) { // SPI interrupt handler
  while (spi->SR & SPI_SR_FRLVL) {
    spiPacket.data[spiPacket.len++] = *((uint8_t *)&spi->DR);
  }
}

uint8_t spiTransaction(const uint8_t data) {
  while (!(spi->SR & SPI_SR_TXE))
    ; // Wait for a free tx slot
  *((uint8_t *)&spi->DR) = data;
  while (!(spi->SR & SPI_SR_RXNE))
    ; // Wait for transaction complete
  return *((uint8_t *)&spi->DR);
}

static void bme280Command(const uint8_t address, const uint8_t data) {
  assert_bme280_cs();
  spiTransaction(address & BME280_W_REG);
  spiTransaction(data);
  deassert_bme280_cs();
}

static void bme280Read(const uint8_t address, uint8_t *data, int len) {
  assert(address != 0xE0);
  assert(address + len <= 0xff);
  assert_bme280_cs();
  spiTransaction(address | BME280_R_REG);
  for (int i = 0; i < len; ++i) {
    data[i] = spiTransaction(BME280_NOP);
  }
  deassert_bme280_cs();
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

  /* ------ Soft reset ------ */
  bme280Command(BME280_RESET, BME280_RESET_WORD);

  /* ------ Test: Check BME280 Registers Defaults------ */
  uint8_t buf[8];
  // Check ID register
  bme280Read(BME280_ID, buf, 1);
  assert(*buf == 0x60);

  // Pressure reading
  // Check sample registers
  bme280Read(BME280_PRESS_MSB, buf, 8);
  assert(buf[0] == 0x80); // Pressure MSB
  assert(buf[1] == 0x00); // Pressure LSB
  assert(buf[2] == 0x00); // Pressure XLSB
  assert(buf[3] == 0x80); // Temp MSB
  assert(buf[4] == 0x00); // Temp LSB
  assert(buf[5] == 0x00); // Temp XLSB
  assert(buf[6] == 0x80); // Hum MSB
  assert(buf[7] == 0x00); // Hum LSB

  /* ------ Write Settings ------ */
  bme280Command(BME280_CTRL_HUM, 1); // Humidity oversampling x1

  // Temperature, Pressure Oversampling x1, Force Sample
  bme280Command(BME280_CTRL_MEAS, 0b00100101);

  // Measure Temperature: 340 uA @ 2x1
  // Measure Pressure: 714 uA @ 2x1 + 0.5
  // Measure Humidity: 350 uA @ 2x1 + 0.5
  // 1 + 2 + 2.5 + 2.5 = 8ms

  // Status should indicate busy measuring
  bme280Read(BME280_STATUS, buf, 1);
  assert(*buf != 0);

  __delay_cycles(64000); // 8ms
  __delay_cycles(24000); // plus some more

  // Should be done measuring
  bme280Read(BME280_STATUS, buf, 1);
  assert(!(*buf & 0b00001000));

  /* ------ Read out data ------ */
  bme280Read(BME280_PRESS_MSB, buf, 8);
  uint16_t up = (buf[0] << 8) | buf[1];
  uint16_t ut = (buf[3] << 8) | buf[4];
  uint16_t uh = (buf[6] << 8) | buf[7];
  assert(up != 0x8000);
  assert(ut != 0x8000);
  assert(uh != 0x8000);
  assert(up > 0);
  assert(ut > 0);
  assert(uh > 0);

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
  Gpio->DIR.WORD = PIN_BME280_CS;
}
