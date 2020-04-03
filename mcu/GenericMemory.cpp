/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <systemc>
#include <tlm>
#include "libs/make_unique.hpp"
#include "mcu/GenericMemory.hpp"
#include "ps/EventLog.hpp"

using namespace sc_core;

GenericMemory::GenericMemory(sc_module_name name, unsigned startAddress,
                             unsigned endAddress, sc_time delay)
    : BusTarget(name, startAddress, endAddress, delay),
      mem(std::make_unique<uint8_t[]>(endAddress - startAddress + 1)),
      m_capacity(endAddress - startAddress + 1) {
  // Register events to be logged
  m_nBytesReadEventId = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " bytes read");
  m_nBytesWrittenEventId = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " bytes written");
}

void GenericMemory::b_transport(tlm::tlm_generic_payload &trans,
                                sc_time &delay) {
  auto addr = trans.get_address();
  auto len = trans.get_data_length();
  auto *data = trans.get_data_ptr();

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    std::memcpy(&mem[addr], data, len);
    m_writeEvent.notify(delay + m_delay);
    m_elog.increment(m_writeEventId);
    m_elog.increment(m_nBytesWrittenEventId, len);
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    std::memcpy(data, &mem[addr], len);
    m_readEvent.notify(delay + m_delay);
    m_elog.increment(m_readEventId);
    m_elog.increment(m_nBytesReadEventId, len);
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  delay += m_delay;
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
