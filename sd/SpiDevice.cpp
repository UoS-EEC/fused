/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sd/SpiDevice.hpp"
#include <systemc>

SpiDevice::SpiDevice(const sc_core::sc_module_name nm)
    : sc_core::sc_module(nm) {
  // Register b_transport method associate with target socket.
  tSpiSocket.register_b_transport(this, &SpiDevice::b_transport);
  // Register process as sensitive to m_spiPayloadReceivedEvent
  SC_METHOD(process);
  sensitive << m_spiPayloadReceivedEvent;
}

void SpiDevice::b_transport(tlm::tlm_generic_payload &trans,
                            sc_core::sc_time &delay) {
  // Read from the received paylaod
  auto *ptr = trans.get_data_ptr();
  auto len = trans.get_data_length();
  auto *spiExtension = trans.get_extension<SpiTransactionExtension>();
  // Response from slave-out shift register
  spiExtension->response = readSoReg();
  std::cout << *spiExtension;
  m_lastTransaction.deep_copy_from(trans);
  // Received payload stored in slave-in shift register
  writeSiReg(trans.get_data_ptr()[0]);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);

  m_spiPayloadReceivedEvent.notify();
}

void SpiDevice::process(void) {
  // Do nothing.
}

void SpiDevice::reset(void) {
  // Clear slave-in & slave-out shift registers
  so_reg = 0x00;
  si_reg = 0x00;
}

uint8_t SpiDevice::readSiReg(void) { return si_reg; }

void SpiDevice::writeSiReg(uint8_t val) { si_reg = val; }

uint8_t SpiDevice::readSoReg(void) { return so_reg; }

void SpiDevice::writeSoReg(uint8_t val) { so_reg = val; }
