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
#include "ps/ConstantCurrentState.hpp"
#include "ps/ConstantEnergyEvent.hpp"

using namespace sc_core;

GenericMemory::GenericMemory(sc_module_name name, unsigned startAddress,
                             unsigned endAddress)
    : BusTarget(name, startAddress, endAddress),
      mem(std::make_unique<uint8_t[]>(endAddress - startAddress + 1)),
      m_capacity(endAddress - startAddress + 1) {}

void GenericMemory::end_of_elaboration() {
  BusTarget::end_of_elaboration();

  // Register events for power model
  m_nBytesWrittenEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "bytes written"));
  m_nBytesReadEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "bytes read"));
  m_offStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "off"));
  m_onStateId = powerModelPort->registerState(
      this->name(), std::make_unique<ConstantCurrentState>("on", 0.0001));

  // Register methods
  SC_METHOD(updatePowerState);
  sensitive << pwrOn.value_changed();
}

void GenericMemory::b_transport(tlm::tlm_generic_payload &trans,
                                sc_time &delay) {
  auto addr = trans.get_address();
  auto len = trans.get_data_length();
  auto *data = trans.get_data_ptr();

  // Forward transaction for analysis by subscribers
  analysisPort.write(trans);

  // Perform transaction
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    std::memcpy(&mem[addr], data, len);
    m_writeEvent.notify(delay + systemClk->getPeriod());
    powerModelPort->reportEvent(m_writeEventId);
    powerModelPort->reportEvent(m_nBytesWrittenEventId, len);
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    std::memcpy(data, &mem[addr], len);
    m_readEvent.notify(delay + systemClk->getPeriod());
    powerModelPort->reportEvent(m_readEventId);
    powerModelPort->reportEvent(m_nBytesReadEventId, len);
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

void GenericMemory::updatePowerState() {
  if (pwrOn.read() == true) {
    powerModelPort->reportState(m_onStateId);
  } else {
    powerModelPort->reportState(m_offStateId);
  }
}

int GenericMemory::size() const { return m_capacity; }
