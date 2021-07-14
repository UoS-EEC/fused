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
#include "include/cm0-fused.h"
#include "mcu/BusTarget.hpp"
#include "mcu/RegisterFile.hpp"

/**
 * @brief The OutputPort class : simple unidirectional output-only "IO" port.
 */
class OutputPort : public BusTarget {
  SC_HAS_PROCESS(OutputPort);

 public:
  /* ------ Ports ------ */
  std::array<sc_core::sc_out<bool>, 32> pins;

  /*------ Methods ------*/
  /**
   * @brief DigitalIo Constructor: initialise registers
   * @param name
   */
  OutputPort(const sc_core::sc_module_name name)
      : BusTarget(name, OUTPORT_BASE, OUTPORT_BASE + 4) {
    // Transport delay
    m_transportDelay = sc_core::sc_time::from_seconds(
        Config::get().getDouble("OutputPortTransportDelay"));

    // Initialize register file
    m_regs.addRegister(OFS_OUTPORT_OUT, 0);

    // Set up methods
    SC_METHOD(reset);
    sensitive << pwrOn;

    SC_THREAD(process);
  };

  /**
   * @brief reset Resets the IO registers to their default power-up values
   * on the rising edge of pwrOn.
   */
  virtual void reset(void) override {
    m_regs.write(OFS_OUTPORT_OUT, 0);
    m_writeEvent.notify(sc_core::SC_ZERO_TIME);  // Update process
  }

 private:
  /* ------ Private variables ------ */
  sc_core::sc_time m_transportDelay;  //! Delay between write and value on pin

  /* ------ Private methods ------ */
  /**
   * @brief process Set all output signals according to register value
   */
  void process() {
    wait(sc_core::SC_ZERO_TIME);

    while (1) {
      wait(m_writeEvent | pwrOn.value_changed_event());
      wait(m_transportDelay);
      unsigned rval = m_regs.read(OFS_OUTPORT_OUT);
      for (unsigned i = 0; i < pins.size(); i++) {
        pins[i].write(rval & (1u << i));
      }
    }
  }
};
