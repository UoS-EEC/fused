/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcu/DummyPeripheral.hpp"

using namespace sc_core;

DummyPeripheral::DummyPeripheral(const sc_module_name name,
                                 const std::vector<unsigned char> retvals,
                                 const unsigned startAddress,
                                 const unsigned endAddress, const sc_time delay)
    : retvals(retvals), BusTarget(name, startAddress, endAddress, delay) {}

DummyPeripheral::DummyPeripheral(const sc_module_name name,
                                 const unsigned startAddress,
                                 const unsigned endAddress, const sc_time delay)
    : retvals(endAddress - startAddress + 1, 0),
      BusTarget(name, startAddress, endAddress, delay) {}

void DummyPeripheral::b_transport(tlm::tlm_generic_payload &trans,
                                  sc_time &delay) {
  auto addr = trans.get_address();

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    // Do nothing, writes are discarded
    m_writeEvent.notify(delay + m_delay);
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    int remaining = trans.get_data_length();
    unsigned char *dst = trans.get_data_ptr();
    while (remaining) {
      *dst++ = retvals[addr++];
      remaining--;
    }
    m_readEvent.notify(delay + m_delay);
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  delay += m_delay;
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned int DummyPeripheral::transport_dbg(tlm::tlm_generic_payload &trans) {
  auto addr = trans.get_address();

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    // Do nothing, writes are discarded
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    int remaining = trans.get_data_length();
    unsigned char *dst = trans.get_data_ptr();
    while (remaining) {
      *dst++ = retvals[addr++];
      remaining--;
    }
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return 0;
}
