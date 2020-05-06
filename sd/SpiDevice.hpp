/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <tlm_utils/simple_target_socket.h>
#include <systemc>
#include <tlm>
#include <vector>
#include "mcu/SpiTransactionExtension.hpp"

/**
 * Abstract class for SPI devices.
 */
class SpiDevice : public sc_core::sc_module {
  SC_HAS_PROCESS(SpiDevice);

 public:
  /* ------ Ports ------ */
  // Supply Power
  sc_core::sc_in<bool> pwrOn{"pwrOn"};
  // Chip select
  sc_core::sc_in<bool> csn{"csn"};
  // TLM socket for SPI
  tlm_utils::simple_target_socket<SpiDevice> tSpiSocket{"tSpiSocket"};

  /* ------ Signals ------ */

  //! Constructor
  explicit SpiDevice(sc_core::sc_module_name nm);

  /**
   * @breif b_transport TLM blocking transaction method.
   * @param trans tlm_generic_payload for SPI packet
   * @param delay
   */
  void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay);

  /**
   * @brief process Entry point for device specific logic.
   */
  virtual void process(void);

  /**
   * @brief reset Reset the device registers, including
   *              the SPI shift registers.
   */
  virtual void reset(void);

  /**
   * @brief n_reg Obtain the number of device memory mapped registers.
   * @return number of memory mapped registers.
   */
  virtual uint32_t n_regs() = 0;

  /**
   * @brief readSiReg Obtain the contents of slave in shift register.
   * @return Contents of shift register.
   */
  uint8_t readSiReg(void);

  /**
   * @brief writeSiReg Write to the slave in shift register.
   * @param Value to write.
   */
  void writeSiReg(uint8_t val);

  /**
   * @brief readSoReg Obtain the contents of slave so shift register.
   * @return Contents of shift register.
   */
  uint8_t readSoReg(void);

  /**
   * @brief writeSoReg Write ot the slave out shift register.
   * @param Value to write.
   */
  void writeSoReg(uint8_t val);

 private:
  uint8_t si_reg;
  uint8_t so_reg;

  tlm::tlm_generic_payload m_lastTransaction;
  uint8_t m_lastPayload;

  sc_core::sc_event m_spiPayloadReceivedEvent{"spiPayloadReceivedEvent"};
};
