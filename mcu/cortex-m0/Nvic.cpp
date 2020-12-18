/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <systemc>
#include <tlm>
#include "mcu/cortex-m0/Nvic.hpp"

using namespace sc_core;

Nvic::Nvic(const sc_module_name name) : BusTarget(name, NVIC_BASE, NVIC_END) {
  // Initialize registers
  uint32_t iprnWriteMask = 0xc0c0c0c0;
  m_regs.addRegister(OFS_NVIC_ISER, 0);
  m_regs.addRegister(OFS_NVIC_ICER, 0);
  m_regs.addRegister(OFS_NVIC_ISPR, 0, RegisterFile::AccessMode::READ_WRITE,
                     0x0);
  m_regs.addRegister(OFS_NVIC_ICPR, 0, RegisterFile::AccessMode::READ_WRITE,
                     0x0);
  m_regs.addRegister(OFS_NVIC_IPR0, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
  m_regs.addRegister(OFS_NVIC_IPR1, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
  m_regs.addRegister(OFS_NVIC_IPR2, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
  m_regs.addRegister(OFS_NVIC_IPR3, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
  m_regs.addRegister(OFS_NVIC_IPR4, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
  m_regs.addRegister(OFS_NVIC_IPR5, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
  m_regs.addRegister(OFS_NVIC_IPR6, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
  m_regs.addRegister(OFS_NVIC_IPR7, 0, RegisterFile::AccessMode::READ_WRITE,
                     iprnWriteMask);
}

void Nvic::end_of_elaboration() {
  BusTarget::end_of_elaboration();
  SC_METHOD(process);
  sensitive << m_writeEvent << active.value_changed_event()
            << returning.value_changed_event() << m_resetEvent;
  for (unsigned i = 0; i < irq.size(); i++) {
    sensitive << irq[i].value_changed_event();
  }

  SC_METHOD(reset);
  sensitive << pwrOn;
  dont_initialize();
}

void Nvic::reset() {
  for (unsigned i = 0; i < m_prevIrq.size(); ++i) {
    m_prevIrq[i] = false;
  }
  m_prevActive = -1;
  m_swClearPending = 0;
  m_swSetPending = 0;

  m_regs.reset();
  m_resetEvent.notify(SC_ZERO_TIME);
}

uint32_t Nvic::writeOneToClear(uint32_t clearbits, uint32_t oldval) {
  return oldval & (~clearbits);
}
uint32_t Nvic::writeOneToSet(uint32_t setbits, uint32_t oldval) {
  return oldval | setbits;
}

void Nvic::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
  BusTarget::b_transport(trans, delay);

  auto addr = trans.get_address();
  auto cmd = trans.get_command();
  auto len = trans.get_data_length();
  uint8_t* data = trans.get_data_ptr();

  switch (addr - (addr % 4)) {  // Aligned address
    case OFS_NVIC_ISER:         // Write one to set, ignore zeros
      if (cmd == tlm::TLM_WRITE_COMMAND) {
        auto result = writeOneToSet(m_regs.read(OFS_NVIC_ISER),
                                    m_regs.read(OFS_NVIC_ICER));
        m_regs.write(OFS_NVIC_ISER, result);
        m_regs.write(OFS_NVIC_ICER, result);
      }
      break;
    case OFS_NVIC_ICER:  // Write one to clear, ignore zeros
      if (cmd == tlm::TLM_WRITE_COMMAND) {
        auto result = writeOneToClear(m_regs.read(OFS_NVIC_ICER),
                                      m_regs.read(OFS_NVIC_ISER));
        m_regs.write(OFS_NVIC_ISER, result);
        m_regs.write(OFS_NVIC_ICER, result);
      }
      break;
    case OFS_NVIC_ISPR:  // Request setting pending status of an IRQ
      if (cmd == tlm::TLM_WRITE_COMMAND) {
        m_swSetPending = Utility::ttohl(Utility::packBytes(data, 4));
      }
      break;
    case OFS_NVIC_ICPR:  // Request clearing pending status of an IRQ
      if (cmd == tlm::TLM_WRITE_COMMAND) {
        m_swClearPending = Utility::ttohl(Utility::packBytes(data, 4));
      }
      break;
    case OFS_NVIC_IPR0:
      break;
    case OFS_NVIC_IPR1:
      break;
    case OFS_NVIC_IPR2:
      break;
    case OFS_NVIC_IPR3:
      break;
    case OFS_NVIC_IPR4:
      break;
    case OFS_NVIC_IPR5:
      break;
    case OFS_NVIC_IPR6:
      break;
    case OFS_NVIC_IPR7:
      break;
    default:
      spdlog::error("SysTick: Invalid address  0x{:08x} accessed.", addr);
      SC_REPORT_FATAL(this->name(), "Invalid address accessed.");
      break;
  }
}

void Nvic::process() {
  // Update pending status for all interrupts & find irq with highest priority
  // (low number => high priority)
  if (pwrOn.read()) {
    auto m_pending = m_regs.read(OFS_NVIC_ISPR);

    int highestPri = 4;
    int highestPriIrq = -1;
    for (int i = 0; i < irq.size(); i++) {
      uint32_t mask = (1u << i);

      // -- Set pending
      // Posedge on irq
      bool edge = irq[i].read() && (!m_prevIrq[i]);

      // irq still requesting when returning from handler
      bool stillRequesting =
          irq[i].read() && (returning.read() == i + NVIC_EXCEPT_ID_BASE);

      // Software set pending
      bool swSet = (m_swSetPending & mask) != 0;

      bool setPend = edge || stillRequesting || swSet;

      // -- Clear pending
      // Software clear: irq level is low && posedge on ICPR
      bool swClear = (irq[i].read() == false) && (m_swClearPending & mask);

      // CPU has started executing the handler
      bool irqActive = (active.read() == i + NVIC_EXCEPT_ID_BASE) &&
                       (active.read() != m_prevActive);

      bool clearPend = swClear || irqActive;

      // -- Resolve pending status
      if (setPend && clearPend) {
        spdlog::warn(
            "{}: irq[{}] has both setPend and clearPend set, this will result "
            "in "
            "IMPLEMENTATION DEFINED behaviour. Will set pending status and "
            "ignore clearPend.",
            this->name());
        clearPend = false;
      }

      if (setPend) {
        m_pending |= mask;
      } else if (clearPend) {
        m_pending &= ~mask;
      }

      // -- Check priority
      uint8_t prio = m_regs.readByte(OFS_NVIC_IPR0 + i) >> 6;
      if ((m_pending & mask) && (prio < highestPri) &&
          (m_regs.read(OFS_NVIC_ISER) & mask)) {
        highestPri = prio;
        highestPriIrq = i;
      }

      m_prevIrq[i] = irq[i].read();
    }

    // -- Set highest-priority pending interrupt
    if (highestPriIrq >= 0) {
      pending.write(NVIC_EXCEPT_ID_BASE + highestPriIrq);
    } else {
      pending.write(highestPriIrq);
    }

    // Update state
    m_regs.write(OFS_NVIC_ISPR, m_pending, true);
    m_regs.write(OFS_NVIC_ICPR, m_pending, true);
    m_prevActive = active.read();
    m_swClearPending = 0;
    m_swSetPending = 0;
  } else {
    pending.write(0);
  }
}

std::ostream& operator<<(std::ostream& os, const Nvic& rhs) {
  // clang-format off
  os << "Nvic: " << rhs.name()
    << "\nclock period " << rhs.systemClk->getPeriod()
    << "\nirq: 0b";

  for (int i = rhs.irq.size() - 1; i > 0; --i) {
    os << (rhs.irq[i].read() ? "1" : "0");
  }

  os << "\nreturning: 0x" << std::hex << rhs.returning.read()
    << "\nactive: 0x" << std::hex << rhs.active.read()
    << "\npending: 0x" << std::hex << rhs.pending.read()
    << std::dec << "\n" << rhs.m_regs;
  // clang-format on
  return os;
}
