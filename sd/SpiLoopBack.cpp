/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "sd/SpiLoopBack.hpp"

using namespace sc_core;

SpiLoopBack::SpiLoopBack(const sc_module_name nm, ChipSelectPolarity polarity)
    : m_chipSelectPolarity(polarity) {
  tSocket.register_b_transport(this, &SpiLoopBack::b_transport);
  SC_METHOD(reset);
  sensitive << nReset;
}

void SpiLoopBack::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
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

void SpiLoopBack::reset(void) {
  // Clear slave-in & slave-out shift registers
  m_SlaveOutRegister = 0;
  m_SlaveInRegister = 0;
}

uint32_t SpiLoopBack::readSlaveIn() const { return m_SlaveInRegister; }

void SpiLoopBack::writeSlaveIn(const uint32_t val) { m_SlaveInRegister = val; }

uint32_t SpiLoopBack::readSlaveOut() const { return m_SlaveOutRegister; }

void SpiLoopBack::writeSlaveOut(const uint32_t val) {
  m_SlaveOutRegister = val;
}

bool SpiLoopBack::enabled() const {
  return ((chipSelect.read() == sc_dt::SC_LOGIC_1) &&
          (m_chipSelectPolarity == ChipSelectPolarity::ActiveHigh)) ||
         ((chipSelect.read() == sc_dt::SC_LOGIC_0) &&
          (m_chipSelectPolarity == ChipSelectPolarity::ActiveLow));
}
