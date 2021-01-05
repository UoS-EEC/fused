/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/msp430fr5xx/DigitalIo.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

DigitalIo::DigitalIo(sc_module_name name, const uint16_t startAddress,
                     const uint16_t endAddress)
    : BusTarget(name, startAddress, endAddress) {
  // Register events
  m_pinPosEdge = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " io_pin_pos");
  m_pinNegEdge = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " io_pin_neg");

  // Initialise register file
  uint16_t endOffset = endAddress - startAddress + 1;
  for (uint16_t i = 0; i < endOffset; i += 2) {
    m_regs.addRegister(i, 0, RegisterFile::AccessMode::READ_WRITE);
  }

  SC_METHOD(reset);
  sensitive << pwrOn;
  dont_initialize();

  SC_METHOD(process);
  sensitive << m_writeEvent << pwrOn;
  for (int i = 0; i < 16; i++) {
    sensitive << pins[i];
  }
}

void DigitalIo::reset(void) {
  m_regs.reset();
  m_lastState = 0;
}

void DigitalIo::process(void) {
  uint16_t dir = m_regs.read(OFS_PADIR);  //  Note: offset is same for all
  uint16_t out = m_regs.read(OFS_PAOUT);  //  ports
  uint16_t ren = m_regs.read(OFS_PAREN);  // Pull-up resistor mode
  uint16_t irqEn = ~m_regs.read(OFS_PASEL0) & ~m_regs.readByte(OFS_PASEL1) &
                   m_regs.read(OFS_PAIE);
  uint16_t irqEdge = m_regs.read(OFS_PAIES);

  // std::cerr << "DigitalIo processing...\n";
  // std::cerr << "DIR: 0x" << std::hex << dir << "\n";
  // std::cerr << "OUT: 0x" << std::hex << out << "\n";
  // std::cerr << "REN: 0x" << std::hex << ren << "\n";
  // std::cerr << "IRQEN: 0x" << std::hex << irqEn << "\n";

  if (pwrOn.read()) {
    for (int i = 0; i < 16; i++) {
      unsigned mask = (1u << i);

      // Read pin as boolean
      bool current;
      if (pins[i].read().is_01()) {
        current = pins[i].read().to_bool();
      } else {  // Read 'Z' and 'X' as 0
        current = false;
        SC_REPORT_WARNING(
            this->name(),
            fmt::format(
                "pin {:d} reads non-binary value {:s}, interpreting as 0.", i,
                pins[i].read().to_char())
                .c_str());
      }

      if ((ren | dir) & mask) {  // If output or Pull-up mode
        // Count edges
        if (!current && (out & mask)) {
          powerModelPort->reportEvent(m_pinPosEdge);
          // std::cerr << "Posedge on pin " << i << '\n';;
        } else if (current && !(out & mask)) {
          powerModelPort->reportEvent(m_pinNegEdge);
          // std::cerr << "Negedge on pin " << i << '\n';;
        }

        // Write to outputs
        pins[i].write(sc_dt::sc_logic((out & mask) != 0));
      }

      if ((~dir) & mask) {  // Input mode
        // Read from inputs
        if (current && !(m_lastState & mask)) {  // High
          m_regs.setBit(OFS_PAIN, i, true);
          m_lastState |= mask;
          if (irqEn & (~irqEdge) & mask) {
            m_regs.setBit(OFS_PAIFG, i, true);
          }
        } else if (!current && (m_lastState & mask)) {  // Low
          m_regs.clearBit(OFS_PAIN, i, true);
          m_lastState &= ~mask;
          if (irqEn & irqEdge & mask) {
            m_regs.setBit(OFS_PAIFG, i, true);
          }
        }
      }

      // Interrupts
      uint16_t irqFlags = m_regs.read(OFS_PAIFG);
      irq[0].write(irqFlags & 0x00ff);
      irq[1].write(irqFlags & 0xff00);
    }
  } else {
    for (int i = 0; i < 16; i++) {
      pins[i].write(sc_dt::sc_logic(false));
    }
  }
  return;
}
