/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <spdlog/spdlog.h>

#include <systemc>

#include "sd/DummySpiDevice.hpp"

using namespace sc_core;

DummySpiDevice::DummySpiDevice(const sc_module_name name) : SpiDevice(name) {
  // Initialise memory mapped control registers.
  // In this case just make a single dummy.
  m_regs.addRegister(0);

  SC_METHOD(process);
  sensitive << m_transactionEvent;
}

void DummySpiDevice::reset(void) {
  // Clear registers
  for (uint16_t i = 0; i < m_regs.size(); i++) {
    m_regs.write(i, 0);
  }
  SpiDevice::reset();
}

void DummySpiDevice::process(void) {
  if (nReset.read()) {
    const auto payload = readSlaveIn();
    spdlog::info("{:s}: @{:s} Received 0x{:08x}", this->name(),
                 sc_time_stamp().to_string(), payload);
    writeSlaveOut(m_regs.read(0) | payload);  // Copy masked payload to output
  }
}
