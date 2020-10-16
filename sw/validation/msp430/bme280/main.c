/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* ------ Includes ----------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>
#include <support.h>

#include "bme280.h"
#include "eusci_b_spi.h"
#include "gpio.h"

/* ------ Function Prototypes -----------------------------------------------*/

// Initialize GPIO settings
static void gpioInit();

// Transmit data, and get response
uint8_t spiTransaction(const uint8_t data);

// Send a command to the bme280
static void bme280Command(const uint8_t address, const uint8_t data);

// Read some data from the bme280 into data
static void bme280Read(const uint8_t address, uint8_t* data, int len);

// Assert c==true, otherwise indicate test fail and stall.
void assert(bool c);

/* ------ Global Variables --------------------------------------------------*/

/* ------ Function Definitions ----------------------------------------------*/

uint8_t spiTransaction(const uint8_t data) {
  EUSCI_B_SPI_transmitData(EUSCI_B1_BASE, data);
  while (!EUSCI_B_SPI_getInterruptStatus(EUSCI_B1_BASE,
                                         EUSCI_B_SPI_RECEIVE_INTERRUPT))
    ;
  uint8_t result = EUSCI_B_SPI_receiveData(EUSCI_B1_BASE);
  return result;
}

static void bme280Command(const uint8_t address, const uint8_t data) {
  GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
  spiTransaction(address & BME280_W_REG);
  spiTransaction(data);
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
}

static void bme280Read(const uint8_t address, uint8_t* data, int len) {
  assert(address != 0xE0);
  assert(address + len <= 0xff);
  GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);
  spiTransaction(address | BME280_R_REG);
  for (int i = 0; i < len; ++i) {
    data[i] = spiTransaction(BME280_NOP);
  }
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
}

int main(void) {
  /* Initialise Clock System and GPIO
   * DCO = 16 MHz; VLO = ~10 kHz
   * ACLK = VLO; MCLK = DCO/2; SMCLK = DCO/2
   * All Ports Output Low
   * Unlock LPM5 Hi-Z
   */
  target_init();

  /* ------ Setup I/O ------ */
  gpioInit();

  /* ------ Setup SPI ------ */
  // Initialize SPI as Master
  EUSCI_B_SPI_initMasterParam param = {0};
  param.selectClockSource = EUSCI_B_SPI_CLOCKSOURCE_SMCLK;
  param.clockSourceFrequency = 8000000;
  param.desiredSpiClock = 1000000;  // 1 Mbps (max. 8 Mbps for radio)
  param.msbFirst = EUSCI_B_SPI_MSB_FIRST;
  param.clockPhase = EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT;
  param.clockPolarity = EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW;
  param.spiMode = EUSCI_B_SPI_3PIN;
  EUSCI_B_SPI_initMaster(EUSCI_B1_BASE, &param);
  EUSCI_B_SPI_enable(EUSCI_B1_BASE);

  /*EUSCI_B_SPI_enableInterrupt(EUSCI_B1_BASE,
   * EUSCI_B_SPI_RECEIVE_INTERRUPT);*/
  /*__enable_interrupt();*/

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
  assert(buf[0] == 0x80);  // Pressure MSB
  assert(buf[1] == 0x00);  // Pressure LSB
  assert(buf[2] == 0x00);  // Pressure XLSB
  assert(buf[3] == 0x80);  // Temp MSB
  assert(buf[4] == 0x00);  // Temp LSB
  assert(buf[5] == 0x00);  // Temp XLSB
  assert(buf[6] == 0x80);  // Hum MSB
  assert(buf[7] == 0x00);  // Hum LSB

  /* ------ Write Settings ------ */
  bme280Command(BME280_CTRL_HUM, 1);  // Humidity oversampling x1

  // Temperature, Pressure Oversampling x1, Force Sample
  bme280Command(BME280_CTRL_MEAS, 0b00100101);

  // Measure Temperature: 340 uA @ 2x1
  // Measure Pressure: 714 uA @ 2x1 + 0.5
  // Measure Humidity: 350 uA @ 2x1 + 0.5
  // 1 + 2 + 2.5 + 2.5 = 8ms

  // Status should indicate busy measuring
  bme280Read(BME280_STATUS, buf, 1);
  assert(*buf != 0);

  __delay_cycles(64000);  // 8ms
  __delay_cycles(24000);  // plus some more

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
  // Assert?

  /*__disable_interrupt();*/
  end_experiment();
  // ^  P1.4 High
}

// USCI_B1 Interrupt Service Routine
/*__attribute__((interrupt(USCI_B1_VECTOR))) void USCI_B1_ISR(void) { ticks++;
 * }*/

void assert(bool c) {
  if (!c) {
    indicate_test_fail();
    while (1)
      ;  // Stall
  }
}

static void gpioInit() {
  /* Select Port 5
   * Set Pin 0,1,2 (C0,C1,C2) to Primary Module Function,
   * UCB1MOSI, UCB1MISO, UCB1CLK.
   */
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
                                             GPIO_PIN0 + GPIO_PIN1 + GPIO_PIN2,
                                             GPIO_PRIMARY_MODULE_FUNCTION);

  // Select Port 3
  // Set Pin 5,6 (B5,B6) as Output
  // CSN, CE
  GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5 + GPIO_PIN6);
  GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);

  // Select Port
  // Set Pin 5 (A12) as Input
  // IRQ
  GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);

  // Expose SMCLK at P3.4 (B4)
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN4,
                                              GPIO_SECONDARY_MODULE_FUNCTION);
}
