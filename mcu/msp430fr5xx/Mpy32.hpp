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

typedef unsigned __int128 uint128_t;

/**
 * @brief The Mpy32 class : 32 bit fix delay multiplier
 * IO outputs
 */
class Mpy32 : public BusTarget {
  SC_HAS_PROCESS(Mpy32);

 public:
  /* ------ Ports ------ */

  /*------ Methods ------*/
  /**
   * @brief Mpy32 Constructor: initialise registers
   * @param name
   * @param delay Bus access delay
   */
  Mpy32(sc_core::sc_module_name name, const uint16_t startAddress,
        const uint16_t endAddress, const sc_core::sc_time delay);

  /**
   * @brief b_transport Blocking reads and writes. Overridden to set state
   * of multiplier according to operand write address
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /**
   * @brief reset Resets ... TODO: fix this description
   */
  virtual void reset(void) override;

 private:
  /* ------ Private variables ------ */

  sc_core::sc_event m_mpyStartEvent{"mpyStartEvent"};
  sc_core::sc_event m_mpyCompleteEvent{
      "mpyCompleteEvent"};  //! Triggered at end of hw multiplication
  /* ------ Private methods ------ */

  /**
   * @brief process Performs multiplication
   * and write to the results register.
   */
  void process();
};
