/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "mcu/msp430fr5xx/eUSCI_B.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"

extern "C" {
#include "device_includes/msp430fr5994.h"
}

using namespace sc_core;

eUSCI_B::eUSCI_B(sc_module_name name, const uint16_t startAddress,
             const uint16_t endAddress, const sc_time delay)
    : BusTarget(name, startAddress, endAddress, delay) {
  // Register events

  // Initialise register file
  uint16_t endOffset = endAddress - startAddress + 1;
  for (uint16_t i = 0; i < endOffset; i += 2) {
    m_regs.addRegister(i, 0, RegisterFile::READ_WRITE);
  }

  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_METHOD(process);
  //sensitive << tx_event << rx_event;
}

void eUSCI_B::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  // Access register file
  BusTarget::b_transport(trans, delay);
  auto addr = trans.get_address();
  
  // Handle serial communication tasks
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    switch (addr) {
      default:
        break;
    }
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    switch (addr) {
      default:
        break;
    }
  }
}

void eUSCI_B::reset(void) {
  if (pwrOn.read()) { // Posedge of pwrOn
  // Reset register file
    for (uint16_t i = 0; i < m_regs.size(); i++) {
      m_regs.write(2 * i, 0);
    }
    m_regs.write(OFS_UCB0CTLW0, 0X01C1);
    m_regs.write(OFS_UCB0IFG, 0x0002);
  }
}

void eUSCI_B::process(void) {
  if (pwrOn.read()) {
  } else {
  }
  return;
}
