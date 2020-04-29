/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <cmath>
#include <systemc>
#include <tlm>
#include "mcu/msp430fr5xx/ClockSystem.hpp"
#include "utilities/Utilities.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

// Base clocks
const sc_time ClockSystem::VLO_PERIOD =
    sc_time(100.0, sc_core::SC_US);  // 10 kHz
const sc_time ClockSystem::MOD_PERIOD =
    sc_time(200.0, sc_core::SC_NS);  // 5 MHz
const sc_time ClockSystem::LFMOD_PERIOD =
    sc_time(128 * 200, sc_core::SC_NS);  // MODCLK / 128

//! DCO clock source selectable frequencies in MHz.
const float ClockSystem::DCO_FREQ_TABLE[] = {
    1.0, 2.67, 3.5, 4.0, 5.33, 8.0,  8.0,  8.0,  // Range select == 0
    1.0, 5.33, 7.0, 8.0, 16.0, 21.0, 24.0, 24.0  // Range select == 1
};

ClockSystem::ClockSystem(sc_module_name name, unsigned startAddress,
                         sc_time delay)
    : BusTarget(name, startAddress, startAddress + OFS_CSCTL6 + 1, delay) {
  SC_METHOD(reset);
  sensitive << pwrOn;
  dont_initialize();

  SC_METHOD(updateClocks);  // Should be initialized
  sensitive << m_writeEvent << pwrOn;

  // Set up register file
  m_regs.addRegister(/*address=*/OFS_CSCTL0,
                     /*value=*/CSCTL0_RST,
                     /*access_mode=*/RegisterFile::READ_WRITE,
                     /*writeMask=*/CSCTL0_MASK);
  m_regs.addRegister(/*address=*/OFS_CSCTL1,
                     /*value=*/CSCTL1_RST,
                     /*access_mode=*/RegisterFile::READ_WRITE,
                     /*writeMask=*/CSCTL1_MASK);
  m_regs.addRegister(/*address=*/OFS_CSCTL2,
                     /*value=*/CSCTL2_RST,
                     /*access_mode=*/RegisterFile::READ_WRITE,
                     /*writeMask=*/CSCTL2_MASK);
  m_regs.addRegister(/*address=*/OFS_CSCTL3,
                     /*value=*/CSCTL3_RST,
                     /*access_mode=*/RegisterFile::READ_WRITE,
                     /*writeMask=*/CSCTL3_MASK);
  m_regs.addRegister(/*address=*/OFS_CSCTL4,
                     /*value=*/CSCTL4_RST,
                     /*access_mode=*/RegisterFile::READ_WRITE,
                     /*writeMask=*/CSCTL4_MASK);
  m_regs.addRegister(/*address=*/OFS_CSCTL5,
                     /*value=*/CSCTL5_RST,
                     /*access_mode=*/RegisterFile::READ_WRITE,
                     /*writeMask=*/CSCTL5_MASK);
  m_regs.addRegister(/*address=*/OFS_CSCTL6,
                     /*value=*/CSCTL6_RST,
                     /*access_mode=*/RegisterFile::READ_WRITE,
                     /*writeMask=*/CSCTL6_MASK);
}

void ClockSystem::reset(void) {
  if (pwrOn.read()) {  // Posedge of pwrOn
    m_regs.write(OFS_CSCTL0, CSCTL0_RST);
    m_regs.write(OFS_CSCTL1, CSCTL1_RST);
    m_regs.write(OFS_CSCTL2, CSCTL2_RST);
    m_regs.write(OFS_CSCTL3, CSCTL3_RST);
    m_regs.write(OFS_CSCTL4, CSCTL4_RST);
    m_regs.write(OFS_CSCTL5, CSCTL5_RST);
    m_regs.write(OFS_CSCTL6, CSCTL6_RST);
    locked = true;
  }
}

void ClockSystem::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  // Write to register file first
  BusTarget::b_transport(trans, delay);

  // Check written values
  auto addr = trans.get_address();
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    if ((addr == OFS_CSCTL0_H) || (addr == OFS_CSCTL0)) {
      if (m_regs.read(OFS_CSCTL0) == CSKEY) {
        // Unlock registers
        locked = false;
      } else {
        spdlog::warn("{}: Wrong password: {}", this->name(),
                     m_regs.read(OFS_CSCTL0));
        // Wrong password -- should issue PUC
      }
    } else if (locked) {
      SC_REPORT_FATAL(this->name(), "Attempt to access locked CS register");
    }
  }

  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

void ClockSystem::updateClocks(void) {
  if (pwrOn.read()) {
    // Start default clocks
    if (vloclk->getPeriod() != VLO_PERIOD) {
      vloclk->setPeriod(VLO_PERIOD);
    }
    if (modclk->getPeriod() != MOD_PERIOD) {
      modclk->setPeriod(MOD_PERIOD);
    }

    // Utility for getting clock dividers
    auto getDivider = [](unsigned regval, int offset) {
      unsigned tmp = (regval & (0b111u << offset)) >> offset;
      if (tmp > 5) {
        return (1u << 5);
      } else {
        return (1u << tmp);
      }
    };

    // DCO
    bool rangeSelect = (m_regs.read(OFS_CSCTL1) & (1u << 6)) >> 6;
    uint8_t freqSelect = (m_regs.read(OFS_CSCTL1) & (0b111u << 1)) >> 1;
    sc_time dcoPeriod = sc_time(
        std::pow(10.0, 3.0) / DCO_FREQ_TABLE[freqSelect + 8 * rangeSelect],
        SC_NS);

    // ACLK
    sc_time sourcePeriod;
    auto select = (m_regs.read(OFS_CSCTL2) & (0b111u << 8)) >> 8;
    switch (select) {
      case 0:  // LFXTCLK if available, otherwise VLOCLK
        sourcePeriod = VLO_PERIOD;
        break;
      case 1:  // VLOCLK
        sourcePeriod = VLO_PERIOD;
        break;
      case 2:  // LFMODCLK
        sourcePeriod = LFMOD_PERIOD;
        break;
      default
          :  // "Reserved, defaults to LFMODCLK, Not recommended for future use"
        spdlog::warn("ACLK clock setting not recommended.");
        sourcePeriod = LFMOD_PERIOD;
        break;
    }

    sc_time aclkPeriod = sourcePeriod * getDivider(m_regs.read(OFS_CSCTL3), 8);
    if (aclk->getPeriod() != aclkPeriod) {
      aclk->setPeriod(aclkPeriod);
    }

    // SMCLK
    select = (m_regs.read(OFS_CSCTL2) & (0b111u << 4)) >> 4;
    switch (select) {
      case 0:  // LFXTCLK if available, otherwise VLOCLK
        sourcePeriod = VLO_PERIOD;
        break;
      case 1:  // VLOCLK
        sourcePeriod = VLO_PERIOD;
        break;
      case 2:  // LFMODCLK
        sourcePeriod = LFMOD_PERIOD;
        break;
      case 3:  // DCOCLK
        sourcePeriod = dcoPeriod;
        break;
      case 4:  // MODCLK
        sourcePeriod = MOD_PERIOD;
        break;
      case 5:  // HFCTCLK if available, otherwise DCOCLK
        sourcePeriod = dcoPeriod;
        break;
      default
          :  // "Reserved, defaults to HFXTCLK, Not recommended for future use"
        spdlog::error("SMCLK clock setting {:d} not supported.", select);
        SC_REPORT_FATAL(this->name(), "Unsupported SMCLK setting.");
        sourcePeriod = dcoPeriod;
        break;
    }

    sc_time smclkPeriod = sourcePeriod * getDivider(m_regs.read(OFS_CSCTL3), 4);
    if (smclk->getPeriod() != smclkPeriod) {
      smclk->setPeriod(smclkPeriod);
    }

    // MCLK
    select = (m_regs.read(OFS_CSCTL2) & (0b111u << 0)) >> 0;
    switch (select) {
      case 0:  // LFXTCLK if available, otherwise VLOCLK
        sourcePeriod = VLO_PERIOD;
        break;
      case 1:  // VLOCLK
        sourcePeriod = VLO_PERIOD;
        break;
      case 2:  // LFMODCLK
        sourcePeriod = LFMOD_PERIOD;
        break;
      case 3:  // DCOCLK
        sourcePeriod = dcoPeriod;
        break;
      case 4:  // MODCLK
        sourcePeriod = MOD_PERIOD;
        break;
      case 5:  // HFCTCLK if available, otherwise DCOCLK
        sourcePeriod = dcoPeriod;
        break;
      default
          :  // "Reserved, defaults to HFXTCLK, Not recommended for future use"
        spdlog::error("MCLK clock setting {:d} not supported.", select);
        SC_REPORT_FATAL(this->name(), "Unsupported MCLK setting.");
        sourcePeriod = dcoPeriod;
        break;
    }

    sc_time mclkPeriod = sourcePeriod * getDivider(m_regs.read(OFS_CSCTL3), 0);
    if (mclk->getPeriod() != mclkPeriod) {
      mclk->setPeriod(mclkPeriod);
    }
  } else if (pwrOn.read() == false) {
    // Off -- stop all clocks
    mclk->setPeriod(SC_ZERO_TIME);
    smclk->setPeriod(SC_ZERO_TIME);
    aclk->setPeriod(SC_ZERO_TIME);
    vloclk->setPeriod(SC_ZERO_TIME);
    modclk->setPeriod(SC_ZERO_TIME);
  }
}
