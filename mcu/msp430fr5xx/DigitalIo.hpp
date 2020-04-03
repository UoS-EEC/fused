/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/EventLog.hpp"

/**
 * @brief The DigitalIo class : model one IO port. For now only used to trace
 * IO outputs
 */
class DigitalIo : public BusTarget {
  SC_HAS_PROCESS(DigitalIo);

 public:
  /* ------ Ports ------ */
  sc_core::sc_out<bool> irq[2];
  sc_core::sc_inout<bool> pins[16];

  /*------ Methods ------*/
  /**
   * @brief DigitalIo Constructor: initialise registers
   * @param name
   * @param delay Bus access delay
   */
  DigitalIo(sc_core::sc_module_name name, const uint16_t startAddress,
            const uint16_t endAddress, const sc_core::sc_time delay);

  /**
   * @brief reset Resets the IO registers to their default power-up values
   * on the rising edge of pwrOn.
   */
  virtual void reset(void) override;

 private:
  /* ------ Private variables ------ */
  EventLog::eventId m_pinPosEdge;
  EventLog::eventId m_pinNegEdge;

  /* ------ Private methods ------ */
  unsigned int countSetBits(uint64_t n);
  /**
   * @brief process Set all output signals according to register value
   */
  void process();
};
