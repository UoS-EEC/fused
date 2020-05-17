/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sd/DummySpiDevice.hpp"

#include <systemc>

DummySpiDevice::DummySpiDevice(const sc_core::sc_module_name name)
    : SpiDevice(name){
  // Initialise memory mapped control registers.
  // In this case just make a single dummy.
  m_regs.addRegister(0,0,RegisterFile::READ_WRITE);
}

void DummySpiDevice::reset(void) {
  // Clear the memory mapped control registers.
  std::cout << "DummySpiDevice Reset" << std::endl;
  for (uint16_t i = 0; i < m_regs.size(); i++) {
    m_regs.write(i,0);
  }
  // Reset SPI shift registers.
  SpiDevice::reset();
}

void DummySpiDevice::process(void) {
  if (pwrOn.read()) {
    // Read from si_reg and decode. Skip for now.
    uint8_t payload = readSiReg();
    std::cout << "DummySpiDevice received: " << (int)payload << " @ "
              << sc_core::sc_time_stamp() << std::endl;
    // Prepare next reponse in so_reg.
    writeSoReg(m_regs.read(0) | payload);
  }
}
