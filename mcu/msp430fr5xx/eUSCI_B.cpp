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
  #include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
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
  sensitive << m_euscibTxEvent;
}

void eUSCI_B::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  // Access register file
  BusTarget::b_transport(trans, delay);
  auto addr = trans.get_address();

  // Handle serial communication tasks
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    switch (addr) {
      case OFS_UCB0CTLW0:
        // Control Register 0
        // Setting UCSWRST resets the eUSCI module.
        if (m_regs.read(OFS_UCB0CTLW0) & UCSWRST) {
          this->reset();
        }
        // Update tlm_generic_paylod extension.
        break;
      case OFS_UCB0BRW:
        // Bit Rate Control Register 1
        // Update tlm_generic_payload clk parameter.
        break;
      case OFS_UCB0STATW:
        // Status Register
        // Setting UCLISTEN enable tx -> rx feedback.
        break;
      case OFS_UCB0RXBUF:
        // Receive Buffer Register. Read only.
        break;
      case OFS_UCB0TXBUF:
        // Transmit Buffer Register 
        // Transmission starts after write.
        // UCTXIFG reset.
        m_euscibTxEvent.notify();
        break;
      case OFS_UCB0IE:
        // Interrupt Enable Register
        // Set UCTXIE and UCRXIE to enable interrupts.
        break;
      case OFS_UCB0IFG:
        // Interrupt Flag Register
        break;
      case OFS_UCB0IV:
        // Interrupt Vector Register. Read only.
        // Access resets the highest-pending interrupt flag.
        // If another flag is set, a second interrupt follows.
        break;
      default:
        break;
    }
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    switch (addr) {
      case OFS_UCB0RXBUF:
        // Reading from the RX buffer clears UCRXIFG and UCOE.
        m_regs.write(OFS_UCB0IFG, m_regs.read(OFS_UCB0IFG) & ~UCRXIFG);
        break;
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
    // Clear the TXIFG flag
    m_regs.write(OFS_UCB0IFG, m_regs.read(OFS_UCB0IFG) & ~(UCTXIFG));
    // Prepare payload object
    tlm::tlm_generic_payload * trans = new tlm::tlm_generic_payload;
    tlm::tlm_command cmd = tlm::TLM_WRITE_COMMAND;   
    uint8_t message = m_regs.read(OFS_UCB0TXBUF);
    sc_time delay = sc_time(0,SC_US);  

    trans->set_command(cmd);
    trans->set_address(0);
    trans->set_data_ptr(&message);
    trans->set_data_length(1);
    trans->set_streaming_width(1);
    trans->set_byte_enable_ptr(0);
    trans->set_dmi_allowed(false);
    trans->set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    
    // Blocking transport call 
    iEusciSocket->b_transport(*trans, delay);
    
    // Check response status
    if (trans->is_response_error())
        SC_REPORT_ERROR("eUSCI_B0", "Response error from TX.");
    
    // Tx Done, Rx Done
    m_regs.write(OFS_UCB0IFG, m_regs.read(OFS_UCB0IFG) | UCTXIFG | UCRXIFG);
    // Process returned message
    m_regs.write(OFS_UCB0RXBUF, *(trans->get_data_ptr()));   

  } else {
  }
  return;
}
