/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include <tlm>
#include "mcu/msp430fr5xx/Frctl_a.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

Frctl_a::Frctl_a(sc_module_name nm, sc_time delay)
    : Frctl_a(nm, delay, FRCTL_A_BASE, FRCTL_A_BASE + OFS_GCCTL1_H) {}

Frctl_a::Frctl_a(sc_module_name nm, sc_time delay, unsigned startAddress,
                 unsigned endAddress)
    : BusTarget(nm, startAddress, endAddress, delay) {
  // Initialize register file
  for (uint16_t i = 0; i < FRCTL_A_SIZE; i += 2) {
    m_regs.addRegister(i, 0, RegisterFile::READ_WRITE);
  }

  SC_METHOD(process);
  sensitive << m_writeEvent;
  dont_initialize();

  SC_METHOD(reset);
  sensitive << pwrOn;
  dont_initialize();
}

void Frctl_a::reset() {
  if (pwrOn.read()) {  // Posedge of pwrOn
    // Reset the register file
    for (uint16_t i = 0; i < FRCTL_A_SIZE; i += 2) {
      m_regs.write(i, 0);
    }
    m_regs.write(OFS_FRCTL0, 0x9600);
    m_regs.write(OFS_GCCTL0, 0x0004);
  }
}

void Frctl_a::process() {
  // Update waitstate
  waitStates.write((m_regs.read(OFS_FRCTL0) >> 4) & 0x000f);
}
