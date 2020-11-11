/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "sd/SpiWire.hpp"

using namespace sc_core;

SpiWire::SpiWire(const sc_module_name nm, ChipSelectPolarity polarity)
    : m_chipSelectPolarity(polarity) {
  tSocket.register_b_transport(this, &SpiWire::b_transport);
  SC_METHOD(reset);
  sensitive << nReset;
}

void SpiWire::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  if (enabled() && nReset.read()) {
    // Read from the received payload & set response
    auto *ptr = trans.get_data_ptr();
    auto len = trans.get_data_length();
    auto *spiExtension = trans.get_extension<SpiTransactionExtension>();
    spiExtension->response = trans.get_data_ptr()[0];
    writeSlaveIn(trans.get_data_ptr()[0]);
    trans.set_response_status(tlm::TLM_OK_RESPONSE);
    m_transactionEvent.notify(delay);
  }
}

void SpiWire::reset(void) {
  // Clear slave-in & slave-out shift registers
  m_SlaveOutRegister = 0;
  m_SlaveInRegister = 0;
}

uint32_t SpiWire::readSlaveIn() const { return m_SlaveInRegister; }

void SpiWire::writeSlaveIn(const uint32_t val) { m_SlaveInRegister = val; }

uint32_t SpiWire::readSlaveOut() const { return m_SlaveOutRegister; }

void SpiWire::writeSlaveOut(const uint32_t val) { m_SlaveOutRegister = val; }

bool SpiWire::enabled() const {
  return ((chipSelect.read() == sc_dt::SC_LOGIC_1) &&
          (m_chipSelectPolarity == ChipSelectPolarity::ActiveHigh)) ||
         ((chipSelect.read() == sc_dt::SC_LOGIC_0) &&
          (m_chipSelectPolarity == ChipSelectPolarity::ActiveLow));
}
