/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "sd/SpiDevice.hpp"

using namespace sc_core;

SpiDevice::SpiDevice(const sc_module_name nm, ChipSelectPolarity polarity)
    : m_chipSelectPolarity(polarity) {
  tSocket.register_b_transport(this, &SpiDevice::b_transport);
  SC_METHOD(reset);
  sensitive << nReset;
}

void SpiDevice::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  if (!chipSelect.read() && nReset.read()) {  // N select is low
    // Read from the received paylaod
    auto *ptr = trans.get_data_ptr();
    auto len = trans.get_data_length();
    auto *spiExtension = trans.get_extension<SpiTransactionExtension>();
    spiExtension->response = readSlaveOut();
    writeSlaveIn(trans.get_data_ptr()[0]);
    trans.set_response_status(tlm::TLM_OK_RESPONSE);

    m_transactionEvent.notify();
  }
}

void SpiDevice::reset(void) {
  // Clear slave-in & slave-out shift registers
  m_SlaveOutRegister = 0;
  m_SlaveInRegister = 0;
}

uint32_t SpiDevice::readSlaveIn() const { return m_SlaveInRegister; }

void SpiDevice::writeSlaveIn(const uint32_t val) { m_SlaveInRegister = val; }

uint32_t SpiDevice::readSlaveOut() const { return m_SlaveOutRegister; }

void SpiDevice::writeSlaveOut(const uint32_t val) { m_SlaveOutRegister = val; }

bool SpiDevice::enabled() const {
  return (chipSelect.read() &&
          (m_chipSelectPolarity == ChipSelectPolarity::ActiveHigh)) ||
         (!chipSelect.read() &&
          (m_chipSelectPolarity == ChipSelectPolarity::ActiveLow));
}
