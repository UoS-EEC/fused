/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/RegisterFile.hpp"

/**
 * @brief The DigitalIo class : model one IO port. For now only used to trace
 * IO outputs
 */
class DigitalIo : public BusTarget {
  SC_HAS_PROCESS(DigitalIo);

 public:
  /* ------ Ports ------ */
  sc_core::sc_out<bool> irq[2];
  std::array<sc_core::sc_inout_resolved, 16> pins;

  /*------ Methods ------*/
  /**
   * @brief DigitalIo Constructor: initialise registers
   * @param name
   */
  DigitalIo(sc_core::sc_module_name name, const uint16_t startAddress,
            const uint16_t endAddress);

  /**
   * @brief reset Resets the IO registers to their default power-up values
   * on the rising edge of pwrOn.
   */
  virtual void reset(void) override;

  /**
   * @brief set up methods, sensitivity, and register power model events and
   * states
   */
  virtual void end_of_elaboration() override;

 private:
  /* ------ Private variables ------ */
  int m_pinPosEdgeId{-1};
  int m_pinNegEdgeId{-1};

  unsigned int m_lastState{0};  // Used to detect input edges

  /* ------ Private methods ------ */
  /**
   * @brief process Set all output signals according to register value
   */
  void process();
};
