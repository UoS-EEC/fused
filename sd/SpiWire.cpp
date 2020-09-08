/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <spdlog/spdlog.h>

#include <systemc>

#include "sd/SpiWire.hpp"

using namespace sc_core;

SpiWire::SpiWire(const sc_module_name name) : SpiDevice(name) {
  // Initialise memory mapped control registers.
  // In this case just make a single dummy.
  m_regs.addRegister(0);

  SC_METHOD(process);
  sensitive << m_transactionEvent;
}

// Override b_transport to immediately echo back received payload
void SpiDevice::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  if (enabled() && nReset.read()) {
    // Read from the received payload & set response
    auto *ptr = trans.get_data_ptr();
    auto len = trans.get_data_length();
    auto *spiExtension = trans.get_extension<SpiTransactionExtension>();
    spiExtension->response = trans.get_data_ptr()[0];  // rx --- tx
    trans.set_response_status(tlm::TLM_OK_RESPONSE);
    m_transactionEvent.notify(delay);
  } else {
    spdlog::warn(
        "{:}:b_transport Transaction while device not active (enabled()={}, "
        "nReset={})",
        this->name(), enabled(), nReset.read());
  }
}

void SpiWire::reset(void) {
  // Clear registers
  for (uint16_t i = 0; i < m_regs.size(); i++) {
    m_regs.write(i, 0);
  }
  SpiDevice::reset();
}

void SpiWire::process(void) {
  if (nReset.read()) {
    const auto payload = readSlaveIn();
    spdlog::info("{:s}: @{:s} Echoing 0x{:08x}", this->name(),
                 sc_time_stamp().to_string(), payload);
  }
}
