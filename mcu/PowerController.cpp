/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcu/PowerController.hpp"

using namespace sc_core;

PowerController::PowerController(const sc_core::sc_module_name name,
                                 const unsigned startAddress,
                                 const unsigned resetValue)
    : BusTarget(name, startAddress, startAddress + 4 - 1) {
  // Build register file
  m_regs.addRegister(RegisterAddress::OUT, resetValue);

  // Set up methods
  SC_METHOD(process);
  sensitive << pwrOn << m_writeEvent << m_resetEvent;

  SC_METHOD(reset);
  sensitive << pwrOn;
}

void PowerController::process() {
  if (pwrOn.read()) {
    const auto ctrl = m_regs.read(RegisterAddress::OUT);
    for (int i = 0; i < out.size(); ++i) {
      const unsigned mask = (1u << i);
      out[i].write((mask & ctrl) != 0);
    }
  } else {
    for (auto &p : out) {
      p.write(false);
    }
  }
}

void PowerController::reset() {
  m_regs.reset();
  m_resetEvent.notify(SC_ZERO_TIME); // Force update
}
