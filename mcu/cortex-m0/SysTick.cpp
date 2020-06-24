/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <iostream>
#include "mcu/cortex-m0/SysTick.hpp"

using namespace sc_core;

SysTick::SysTick(const sc_module_name name, const sc_time delay)
    : BusTarget(name, SYST_BASE, SYST_END, delay) {
  // Methods
  SC_METHOD(process);
  sensitive << m_expiredEvent;
  dont_initialize();

  SC_METHOD(irqControl);
  sensitive << returning_exception << m_updateIrqEvent;
  dont_initialize();

  SC_METHOD(reset);
  sensitive << pwrOn;
  dont_initialize();

  // Set up register file
  m_regs.addRegister(/*Address=*/OFS_SYST_CSR,
                     /*Default value=*/SYST_CSR_CLKSOURCE,
                     /*accessMode=*/RegisterFile::AccessMode::READ_WRITE,
                     /*writeMask=*/0x3);
  m_regs.addRegister(/*Address=*/OFS_SYST_RVR,
                     /*Default value=*/0x00aaaa,  // UNDEF
                     /*accessMode=*/RegisterFile::AccessMode::READ_WRITE,
                     /*writeMask=*/0x00ffffff);  // 24 bit
  m_regs.addRegister(/*Address=*/OFS_SYST_CVR,
                     /*Default value=*/0x00aaaa,  // UNDEF
                     /*accessMode=*/RegisterFile::AccessMode::READ_WRITE,
                     /*writeMask=*/0x00ffffff);  // Writes should clear value
  m_regs.addRegister(
      /*Address=*/OFS_SYST_CALIB,
      /*Default value=*/0,  //! Overwritten in end_of_elaboration
      /*accessMode=*/RegisterFile::AccessMode::READ,
      /*writeMask=*/0x0);  // Writes should clear value
}

void SysTick::end_of_elaboration() {
  const unsigned tenms =
      static_cast<unsigned>(sc_time(10, SC_MS) / clk->getPeriod()) - 1;
  m_regs.write(OFS_SYST_CALIB, SYST_CALIB_NOREF | tenms, /*force=*/true);
}

void SysTick::reset(void) {
  m_regs.reset();
  const unsigned tenms =
      static_cast<unsigned>(sc_time(10, SC_MS) / clk->getPeriod()) - 1;
  m_regs.write(OFS_SYST_CALIB, SYST_CALIB_NOREF | tenms, /*force=*/true);
  m_expiredEvent.cancel();
  m_updateIrqEvent.cancel();
  m_setIrq = false;
}

void SysTick::process() {
  m_regs.write(OFS_SYST_CVR, m_regs.read(OFS_SYST_RVR));  // Reload
  if (m_regs.read(OFS_SYST_CVR) > 0) {
    m_expiredEvent.notify(clk->getPeriod() * (m_regs.read(OFS_SYST_CVR) + 1));
  }
  // Set COUNTFLAG
  m_regs.setBitMask(OFS_SYST_CSR, SYST_CSR_COUNTFLAG);
  if (m_regs.read(OFS_SYST_CSR) & SYST_CSR_TICKINT) {
    // Tick Interrupt enabled
    m_setIrq = true;
    m_updateIrqEvent.notify(SC_ZERO_TIME);
  }
  m_regs.setBitMask(OFS_SYST_CSR, SYST_CSR_COUNTFLAG, /*force=*/true);
  m_lastTick = sc_time_stamp();
}

void SysTick::irqControl() {
  if (m_setIrq == true) {
    irq.write(true);
  } else if (returning_exception.read() == SYST_EXCEPT_ID) {
    irq.write(false);
  }
  m_setIrq = false;
}

int SysTick::calcCVR() const {
  if (clk->getPeriod() == SC_ZERO_TIME) {
    // Input clock inactive
    return 0;
  }
  return m_regs.read(OFS_SYST_RVR) -
         static_cast<int>((sc_time_stamp() - m_lastTick) / clk->getPeriod());
}

void SysTick::b_transport(tlm::tlm_generic_payload& trans, sc_time& delay) {
  bool wasEnabled = m_regs.read(OFS_SYST_CSR) & SYST_CSR_ENABLE;
  BusTarget::b_transport(trans, delay);
  bool isEnabled = m_regs.read(OFS_SYST_CSR) & SYST_CSR_ENABLE;

  auto addr = trans.get_address();
  auto cmd = trans.get_command();
  auto len = trans.get_data_length();
  uint8_t* data = trans.get_data_ptr();

  switch (addr - (addr % 4)) {
    case OFS_SYST_CSR:
      if (cmd == tlm::TLM_READ_COMMAND) {
        m_regs.clearBitMask(OFS_SYST_CSR, SYST_CSR_COUNTFLAG, true);
      } else if (cmd == tlm::TLM_WRITE_COMMAND) {
        if (wasEnabled && (!isEnabled)) {
          // Disable
          m_expiredEvent.cancel();
          m_regs.write(OFS_SYST_CVR, calcCVR());
        } else if ((!wasEnabled) && isEnabled) {
          // Enable
          m_lastTick = sc_time_stamp();
          sc_time nextTick =
              (m_regs.read(OFS_SYST_CVR) == 0)
                  ? clk->getPeriod() * (m_regs.read(OFS_SYST_RVR) + 1)
                  : clk->getPeriod() * (m_regs.read(OFS_SYST_CVR) + 1);
          m_expiredEvent.notify(nextTick);
        }
      }
      break;
    case OFS_SYST_RVR:
      // Do nothing
      break;
    case OFS_SYST_CVR:
      if (cmd == tlm::TLM_WRITE_COMMAND) {
        // Writes to CVR clears value & COUNTFLAG
        m_regs.write(OFS_SYST_CVR, 0);
        m_regs.clearBitMask(OFS_SYST_CSR, SYST_CSR_COUNTFLAG, true);
        if (isEnabled) {  // Reset next expiration event
          m_expiredEvent.cancel();
          m_expiredEvent.notify(clk->getPeriod() *
                                (1 + m_regs.read(OFS_SYST_RVR)));
        }
        // Signal reset
      } else if (cmd == tlm::TLM_READ_COMMAND) {
        if (!isEnabled) {
          m_regs.read(addr, data, len);  // Return stored CVR
        } else {
          // return calculated CVR
          uint32_t tmp = Utility::htotl(calcCVR());
          Utility::unpackBytes(data, tmp, 4);
        }
      }
      break;
    case OFS_SYST_CALIB:
      break;
    default:
      spdlog::error("SysTick: Invalid address  0x{:08x} accessed.", addr);
      SC_REPORT_FATAL(this->name(), "Invalid address accessed.");
      break;
  }
}

std::ostream& operator<<(std::ostream& os, const SysTick& rhs) {
  bool isEnabled = rhs.m_regs.read(OFS_SYST_CSR) & SYST_CSR_ENABLE;

  // clang-format off
  os << "<SysTick> " << rhs.name()
    << "\nClock period " << rhs.clk->getPeriod()
    << "\nirq: " << rhs.irq.read()
    << "\nreturning_exception: " << rhs.returning_exception.read()
    << "\n SYST_CSR 0x" << std::hex << rhs.m_regs.read(OFS_SYST_CSR)
    << "\n SYST_RVR 0x" << std::hex << rhs.m_regs.read(OFS_SYST_RVR)
    << "\n SYST_CVR 0x" << std::hex << (isEnabled ? rhs.calcCVR() : rhs.m_regs.read(OFS_SYST_CVR))
    << "\n SYST_CALIB 0x" << std::hex << rhs.m_regs.read(OFS_SYST_CALIB) << "\n";
  // clang-format on
  return os;
}
