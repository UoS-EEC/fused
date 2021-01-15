/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include "BusTarget.hpp"
#include "libs/make_unique.hpp"
#include "ps/ConstantEnergyEvent.hpp"

using namespace sc_core;

BusTarget::BusTarget(const sc_module_name name, const unsigned startAddress,
                     const unsigned endAddress)
    : sc_module(name),
      tSocket("tSocket"),
      m_startAddress(startAddress),
      m_endAddress(endAddress) {
  sc_assert(startAddress <= endAddress);
  tSocket.bind(*this);
}

void BusTarget::end_of_elaboration() {
  m_readEventId = powerModelPort->registerEvent(
      this->name(), std::make_unique<ConstantEnergyEvent>("read"));
  m_writeEventId = powerModelPort->registerEvent(
      this->name(), std::make_unique<ConstantEnergyEvent>("write"));
}

void BusTarget::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  auto addr = trans.get_address();
  auto len = trans.get_data_length();
  uint8_t *data = trans.get_data_ptr();

  // Perform transaction
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    m_regs.write(addr, data, len);
    m_writeEvent.notify(delay + systemClk->getPeriod());
    powerModelPort->reportEvent(m_writeEventId);
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    m_regs.read(addr, data, len);
    m_readEvent.notify(delay + systemClk->getPeriod());
    powerModelPort->reportEvent(m_readEventId);
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  delay += systemClk->getPeriod();
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned int BusTarget::transport_dbg(tlm::tlm_generic_payload &trans) {
  size_t len = trans.get_data_length();
  auto addr = trans.get_address();
  uint8_t *data = trans.get_data_ptr();

  // Perform transaction
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    m_regs.write(addr, data, len);
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    m_regs.read(addr, data, len);
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return trans.get_data_length();
}

std::ostream &operator<<(std::ostream &os, const BusTarget &rhs) {
  os << "<BusTarget> " << rhs.name() << "\n"
     << "StartAddress: 0x" << std::hex << rhs.startAddress() << '\n'
     << "EndAddress: 0x" << std::hex << rhs.endAddress() << '\n'
     << rhs.m_regs;
  return os;
}
