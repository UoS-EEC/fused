/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/RegisterFile.hpp"
#include "mcu/msp430fr5xx/TimerA.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

const int TimerA::ACLK_SEL = 0;
const int TimerA::SMCLK_SEL = 1;

TimerA::TimerA(sc_module_name name, unsigned startAddress, sc_time delay)
    : BusTarget(name, startAddress, startAddress + OFS_TA1IV + 1, delay) {
  // Bind submodules
  // {aclk / smlclk} -> mux -> div -> timerClock
  clkMux.inClk[ACLK_SEL].bind(aclk);
  clkMux.inClk[SMCLK_SEL].bind(smclk);
  clkMux.outClk.bind(muxOut);
  clkMux.sel.bind(clkMuxSelect);

  clkDiv.inClk.bind(muxOut);
  clkDiv.divIn.bind(clkDivAmount);
  clkDiv.outClk.bind(timerClock);

  // Set up register file
  m_regs.addRegister(OFS_TA1CTL, 0);
  m_regs.addRegister(OFS_TA1CCTL0, 0);
  m_regs.addRegister(OFS_TA1CCTL1, 0);
  m_regs.addRegister(OFS_TA1CCTL2, 0);
  // m_regs.addRegister(OFS_TA1CCTL3, 0); // Not implemented in hw
  // m_regs.addRegister(OFS_TA1CCTL4, 0); // Not implemented in hw
  // m_regs.addRegister(OFS_TA1CCTL5, 0); // Not implemented in hw
  // m_regs.addRegister(OFS_TA1CCTL6, 0); // Not implemented in hw
  m_regs.addRegister(OFS_TA1R, 0);
  m_regs.addRegister(OFS_TA1CCR0, 0);
  m_regs.addRegister(OFS_TA1CCR1, 0);
  m_regs.addRegister(OFS_TA1CCR2, 0);
  // m_regs.addRegister(OFS_TA1CCR3, 0); // Not implemented in hw
  // m_regs.addRegister(OFS_TA1CCR4, 0); // Not implemented in hw
  // m_regs.addRegister(OFS_TA1CCR5, 0); // Not implemented in hw
  // m_regs.addRegister(OFS_TA1CCR6, 0); // Not implemented in hw
  m_regs.addRegister(OFS_TA1IV, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(OFS_TA1EX0, 0);

  // Register events
  m_triggerEvent = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " triggered");
}

void TimerA::end_of_elaboration() {
  // Register SC_METHODS here (after events have been constructed)
  SC_METHOD(process);
  sensitive << timerClock << ira;

  SC_METHOD(updateClkSource);
  sensitive << sourceChangeEvent;

  SC_METHOD(reset);
  sensitive << pwrOn;
  dont_initialize();
}

void TimerA::reset(void) { m_regs.reset(); }

void TimerA::process(void) {
  if (pwrOn.read()) {
    // Operation
    bool stopped;
    unsigned crntCnt = m_regs.read(OFS_TA1R);
    unsigned modeControl = (m_regs.read(OFS_TA1CTL) & (0b11u << 4)) >> 4;
    switch (modeControl) {  // Mode control
      case 0:               // Stop mode: timer is halted.
        stopped = true;
        break;
      case 1:  // Up mode: timer counts up to TAxCCR0
        stopped = false;
        if (crntCnt < m_regs.read(OFS_TA1CCR0)) {
          crntCnt++;
          m_regs.write(OFS_TA1R, crntCnt);
        } else {
          m_regs.setBit(OFS_TA1CTL, 0);  // Set IFG
          m_regs.write(OFS_TA1R, 0);     // Clear count
          EventLog::getInstance().increment(m_triggerEvent);
        }
        break;
      case 2:  // Continuous mode: timer counts up to 0xffff
        stopped = false;
        if (crntCnt < 0xffff) {
          crntCnt++;
          m_regs.write(OFS_TA1R, crntCnt);
        } else {
          m_regs.setBitMask(OFS_TA1CTL, TAIFG);
          m_regs.write(OFS_TA1R, 0);  // Clear count
          EventLog::getInstance().increment(m_triggerEvent);
        }
        break;
      case 3:  // Up/down mode: timer counts up to TAxCCR0 then down to 0
        stopped = false;
        if (direction && (crntCnt < m_regs.read(OFS_TA1CCR0))) {
          crntCnt++;
          m_regs.write(OFS_TA1R, crntCnt);
          direction = !(crntCnt < m_regs.read(OFS_TA1CCR0));
        } else if ((crntCnt > 0) && (!direction)) {
          crntCnt--;
          m_regs.write(OFS_TA1R, crntCnt);
          direction = (crntCnt == 0);
        }

        if (crntCnt == 0) {
          m_regs.setBitMask(OFS_TA1CTL, TAIFG);
          EventLog::getInstance().increment(m_triggerEvent);
        }
        break;
    }

    if (m_regs.read(OFS_TA1CTL) & TACLR) {  // Clear state (not settings)
      m_regs.write(OFS_TA1CTL, 0);
      direction = true;
      // reset clock dividers (count, not setting)
      // clkDiv->resetCnt();
      // Ignored for now
    }

    // Clear interrupt flag if interrupt request accepted (acknowledged)
    const bool irqEnabled =
        (m_regs.read(OFS_TA1CTL) & TAIE) || (m_regs.read(OFS_TA1CCTL0) & CCIE);
    if (ira.read()) {
      m_regs.clearBitMask(OFS_TA1CTL, TAIFG);  // Auto-cleared
      irq.write(false);
    } else if (irqEnabled) {
      // Set IRQ if interrupt flag set
      irq.write(m_regs.read(OFS_TA1CTL) & TAIFG);
    }

    // Set DMA trigger if interrupts diabled & interrupt flag set
    if (!irqEnabled && (m_regs.read(OFS_TA1CTL) & TAIFG)) {
      dmaTrigger.write(true);
      m_regs.clearBitMask(OFS_TA1CTL, TAIFG);  // Auto-cleared
      spdlog::info("{}: @{:s} DMA trigger", this->name(),
                   sc_time_stamp().to_string());
    } else {
      dmaTrigger.write(false);
    }

    if (stopped) {
      next_trigger(m_writeEvent);
    } else {
      next_trigger(timerClock.default_event() | ira.default_event());
    }
  }
}

void TimerA::updateClkSource() {
  // Mux
  auto tassel = m_regs.read(OFS_TA1CTL) & TASSEL;

  switch (tassel) {  // Clock source select
    case TASSEL_0:   // TAxCLK
      clkMuxSelect.write(SMCLK_SEL);
      spdlog::warn(
          "{:s}: TAxCLK clock source not implemented, defaulting to SMCLK",
          this->name());
      break;
    case TASSEL_1:  // ACLK
      clkMuxSelect.write(ACLK_SEL);
      break;
    case TASSEL_2:  // SMCLK
      clkMuxSelect.write(SMCLK_SEL);
      break;
    case TASSEL_3:  // INCLK
      clkMuxSelect.write(SMCLK_SEL);
      spdlog::warn(
          "{:s}: INCLK clock source not implemented, defaulting to SMCLK",
          this->name());
      break;
  }

  // Divider
  auto div = (1u << ((m_regs.read(OFS_TA1CTL) & (0x11 << 6u)) >> 6u));  // TAID
  div *= (1 + (m_regs.read(OFS_TA1EX0) & TAIDEX));  // TAIDEX
  clkDivAmount.write(div);
}

void TimerA::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  BusTarget::b_transport(trans, delay);

  uint16_t addr = (uint16_t)trans.get_address();

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    if (addr == OFS_TA1CTL) {
      sourceChangeEvent.notify(delay);
    }
  }
}
