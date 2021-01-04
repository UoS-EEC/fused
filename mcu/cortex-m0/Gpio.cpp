/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <systemc>
#include "include/cm0-fused.h"
#include "mcu/cortex-m0/Gpio.hpp"

using namespace sc_core;

Gpio::Gpio(const sc_core::sc_module_name name)
    : BusTarget(name, GPIO_BASE, GPIO_BASE + GPIO_SIZE - 1) {
  // Initialize register file
  m_regs.addRegister(OFS_GPIO_DATA);
  m_regs.addRegister(OFS_GPIO_DIR);
  m_regs.addRegister(OFS_GPIO_IE);
  m_regs.addRegister(OFS_GPIO_IFG, 0, RegisterFile::AccessMode::READ);

  // Get event IDs
  m_pinPosEdge = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " negedge");
  m_pinNegEdge = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " posedge");
};

void Gpio::before_end_of_elaboration() {
  // Set up methods
  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_METHOD(process);
  sensitive << clk << pwrOn.pos();

  SC_METHOD(irqControl);
  sensitive << active_exception << m_updateIrqEvent;
  dont_initialize();
}

void Gpio::reset(void) {
  m_regs.reset();
  m_lastState = 0;
  m_writeEvent.notify(sc_core::SC_ZERO_TIME);
  m_setIrq = false;
  m_updateIrqEvent.notify(SC_ZERO_TIME);
}

void Gpio::process(void) {
  unsigned dir = m_regs.read(OFS_GPIO_DIR);    // Pin direction (in=0/out=1)
  unsigned data = m_regs.read(OFS_GPIO_DATA);  // Pin states
  unsigned ie = m_regs.read(OFS_GPIO_IE);      // Interrupt enable

  if (pwrOn.read()) {
    for (int i = 0; i < pins.size(); i++) {
      unsigned mask = (1u << i);  // Pin mask

      // Read pin as boolean
      bool current;
      if (pins[i].read().is_01()) {
        current = pins[i].read().to_bool();
      } else {  // Read 'Z' and 'X' as 0
        current = false;
        /*
        SC_REPORT_WARNING(
            this->name(),
            fmt::format(
                "pin {:d} reads non-binary value {:}, interpreting as 0.", i,
                pins[i].read().to_char())
                .c_str());
                */
      }

      if (dir & mask) {  // If output mode
        // Count edges
        if (!current && (data & mask)) {  // Posedge
          powerModelEventPort->reportEvent(m_pinPosEdge);
          spdlog::info("{:s}: @{:s} posedge on pin {:d}", this->name(),
                       sc_time_stamp().to_string(), i);
        } else if (current && !(data & mask)) {  // Negedge
          powerModelEventPort->reportEvent(m_pinNegEdge);
          spdlog::info("{:s}: @{:s} negedge on pin {:d}", this->name(),
                       sc_time_stamp().to_string(), i);
        }

        // Write to outputs
        pins[i].write(sc_dt::sc_logic((data & mask) != 0));
      } else {  // Input mode
        // Read from inputs
        if (current && !(m_lastState & mask)) {  // Posedge
          spdlog::info("{:s}: @{:s} posedge on pin {:d}", this->name(),
                       sc_time_stamp().to_string(), i);
          m_regs.setBit(OFS_GPIO_DATA, i, true);
          m_lastState |= mask;
          if (ie & mask) {
            m_regs.setBit(OFS_GPIO_IFG, i, true);
            m_setIrq = true;
            m_updateIrqEvent.notify(SC_ZERO_TIME);
          }
        } else if (!current && (m_lastState & mask)) {  // Negedge
          spdlog::info("{:s}: @{:s} negedge on pin {:d}", this->name(),
                       sc_time_stamp().to_string(), i);
          m_regs.clearBit(OFS_GPIO_DATA, i, true);
          m_lastState &= ~mask;
          /* // Irq only on posedge for now
          if (irqEn & irqEdge & mask) {
            m_regs.setBit(OFS_PAIFG, i, true);
          }
          */
        }
      }
    }
  } else {
    for (int i = 0; i < pins.size(); i++) {
      pins[i].write(sc_dt::SC_LOGIC_Z);
    }
  }
  return;
}

void Gpio::irqControl() {
  if (pwrOn.read() == false) {
    irq.write(false);
    m_setIrq = false;
    return;
  }
  if (m_setIrq && (!irq.read())) {
    spdlog::info("{:s}: @{:s} interrupt request", this->name(),
                 sc_time_stamp().to_string());
    irq.write(true);
  } else if ((active_exception.read() - 16) == GPIO_EXCEPT_ID) {
    spdlog::info("{:s}: @{:s} interrupt request cleared.", this->name(),
                 sc_time_stamp().to_string());
    irq.write(false);
  }
  m_setIrq = false;
}

std::ostream& operator<<(std::ostream& os, const Gpio& rhs) {
  // clang-format off
  os << "<Gpio> " << rhs.name()
    << "\n\tI/O clock period " << rhs.clk->getPeriod()
    << "\n\tirq: " << rhs.irq.read()
    << fmt::format("\n\tDATA\t 0x{:04x}", rhs.m_regs.read(OFS_GPIO_DATA))
    << fmt::format("\n\tDIR\t 0x{:04x}", rhs.m_regs.read(OFS_GPIO_DIR))
    << fmt::format("\n\tIE\t 0x{:04x}", rhs.m_regs.read(OFS_GPIO_IE))
    << fmt::format("\n\tIFG\t 0x{:04x}", rhs.m_regs.read(OFS_GPIO_IFG))
    << "\n";
  return os;
  // clang-format on
}
