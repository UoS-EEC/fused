/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <memory>
#include <systemc>
#include <tlm>
#include "libs/make_unique.hpp"
#include "mcu/GenericMemory.hpp"
#include "ps/ConstantEnergyEvent.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"

using namespace sc_core;

GenericMemory::GenericMemory(sc_module_name name, unsigned startAddress,
                             unsigned endAddress)
    : BusTarget(name, startAddress, endAddress),
      mem(std::make_unique<uint8_t[]>(endAddress - startAddress + 1)),
      m_capacity(endAddress - startAddress + 1) {}

void GenericMemory::end_of_elaboration() {
  BusTarget::end_of_elaboration();

  // Register events for power model
  m_nBytesWrittenEventId =
      powerModelEventPort->registerEvent(std::make_unique<ConstantEnergyEvent>(
          fmt::format(FMT_STRING("{:s} bytes written"), this->name())));
  m_nBytesReadEventId =
      powerModelEventPort->registerEvent(std::make_unique<ConstantEnergyEvent>(
          fmt::format(FMT_STRING("{:s} bytes read"), this->name())));
}

void GenericMemory::b_transport(tlm::tlm_generic_payload &trans,
                                sc_time &delay) {
  auto addr = trans.get_address();
  auto len = trans.get_data_length();
  auto *data = trans.get_data_ptr();

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    std::memcpy(&mem[addr], data, len);
    m_writeEvent.notify(delay + systemClk->getPeriod());
    powerModelEventPort->write(m_writeEventId);
    powerModelEventPort->write(m_nBytesWrittenEventId, len);
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    std::memcpy(data, &mem[addr], len);
    m_readEvent.notify(delay + systemClk->getPeriod());
    powerModelEventPort->write(m_readEventId);
    powerModelEventPort->write(m_nBytesReadEventId, len);
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  delay += systemClk->getPeriod();
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned int GenericMemory::transport_dbg(tlm::tlm_generic_payload &trans) {
  auto addr = trans.get_address();
  auto len = trans.get_data_length();
  auto *data = trans.get_data_ptr();

  // Perform transaction
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    std::memcpy(&mem[addr], data, len);
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    std::memcpy(data, &mem[addr], len);
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return len;
}

int GenericMemory::size() const { return m_capacity; }
