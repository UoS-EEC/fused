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
#include "mcu/BusTarget.hpp"
#include "mcu/SpiTransactionExtension.hpp"

/**
 * Base class for SPI devices.
 */
class SpiDevice : {
  SC_HAS_PROCESS(SpiDevice);

 public:
  /* ------ Ports ------ */
  sc_core::sc_in<bool> chipSelect{"chipSelect"};

  /* ------ Signals ------ */

  /* ------ Public types ------ */
  enum class ChipSelectPolarity { ActiveHigh, ActiveLow };

  /* ------ Public methods ------ */

  //! Constructor
  explicit SpiDevice(
      sc_core::sc_module_name nm,
      ChipSelectPolarity polarity = ChipSelectPolarity::ActiveHigh);

  /**
   * @brief b_transport TLM blocking transaction method.
   * @param trans tlm_generic_payload for SPI packet
   * @param delay
   */
  void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay);

 protected:
  /* ------ Protected methods ------ */

  /**
   * @brief reset Reset the device registers, including
   *              the SPI shift registers.
   */
  virtual void reset() = 0;

  /**
   * @brief check whether enabled or not according to chipSelect signal and
   * chipSelectPolarity.
   */
  virtual void enabled() const;

  /**
   * @brief readSlaveIn Obtain the contents of slave in shift register.
   * @return Contents of shift register.
   */
  uint32_t readSlaveIn() const;

  /**
   * @brief writeSlaveIn Write to the slave in shift register.
   * @param Value to write.
   */
  void writeSlaveIn(const uint32_t val);

  /**
   * @brief readSlaveOut Obtain the contents of slave so shift register.
   * @return Contents of shift register.
   */
  uint32_t readSlaveOut() const;

  /**
   * @brief writeSlaveOut Write to the slave out shift register.
   * @param Value to write.
   */
  void writeSlaveOut(const uint32_t val);

  /* ------ Protected variables ------ */
  const m_chipSelectPolarity{ChipSelectPolarity::ActiveHigh};
  sc_core::sc_event m_transactionEvent{"m_transactionEvent"};

 private:
  uint32_t m_SlaveInRegister;
  uint32_t m_SlaveOutRegister;
};
