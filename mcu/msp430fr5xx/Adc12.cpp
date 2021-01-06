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
#include "libs/make_unique.hpp"
#include "mcu/ClockDivider.hpp"
#include "mcu/ClockMux.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/msp430fr5xx/Adc12.hpp"
#include "ps/ConstantCurrentState.hpp"
#include "ps/ConstantEnergyEvent.hpp"
#include "utilities/Utilities.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

#define ADC12_B_SIZE (OFS_ADC12MEM31 + 2)

using namespace sc_core;

const int Adc12::MODCLK_SEL = 0;
const int Adc12::ACLK_SEL = 1;
const int Adc12::MCLK_SEL = 2;
const int Adc12::SMCLK_SEL = 3;

Adc12::Adc12(const sc_module_name name)
    : BusTarget(name, ADC12_B_BASE, ADC12_B_BASE + OFS_ADC12MEM31_H) {
  // Bind submodules
  // {aclk / smlclk} -> mux -> div -> timerClock
  clkMux.inClk[MODCLK_SEL].bind(modclk);
  clkMux.inClk[ACLK_SEL].bind(aclk);
  clkMux.inClk[MCLK_SEL].bind(mclk);
  clkMux.inClk[SMCLK_SEL].bind(smclk);
  clkMux.outClk.bind(muxOut);
  clkMux.sel.bind(clkMuxSelect);
  clkDiv.inClk.bind(muxOut);
  clkDiv.divIn.bind(clkDivAmount);
  clkDiv.outClk.bind(samplingClock);

  // Construct register file
  for (uint16_t i = 0; i < ADC12_B_SIZE; i += 2) {
    if (i == OFS_ADC12CTL2) {
      m_regs.addRegister(i, 0x20, RegisterFile::AccessMode::READ_WRITE);
    } else if (i == OFS_ADC12HI) {
      m_regs.addRegister(i, 0xfff, RegisterFile::AccessMode::READ_WRITE);
    } else if ((i >= OFS_ADC12MEM0) && (i <= OFS_ADC12MEM31)) {
      // Write arbitrary value to memory registers (undefined value at reset)
      m_regs.addRegister(i, 0xaaaa, RegisterFile::AccessMode::READ_WRITE);
    } else {
      m_regs.addRegister(i, 0, RegisterFile::AccessMode::READ_WRITE);
    }
  }
}

void Adc12::end_of_elaboration() {
  BusTarget::end_of_elaboration();
  // Register power modelling states & events
  m_offStateId = powerModelPort->registerState(
      this->name(), std::make_unique<ConstantCurrentState>("off"));
  m_onStateId = powerModelPort->registerState(
      this->name(), std::make_unique<ConstantCurrentState>("on"));

  m_sampleEventId =
      powerModelPort->registerEvent(std::make_unique<ConstantEnergyEvent>(
          std::string(this->name()) + " sample"));

  // Register SC_METHODs here (after construction)
  SC_METHOD(process);
  sensitive << samplingClock;
  dont_initialize();

  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_METHOD(updateClkSource);
  sensitive << samplingClockUpdateEvent;
}

void Adc12::reset(void) {
  if (pwrOn.read()) {  // Posedge of pwrOn
    // Reset the register file
    m_regs.reset();
    modeEvent.cancel();              // Cancel pending events
    modeEvent.notify(SC_ZERO_TIME);  // initialize process
  } else {                           // Negedge of pwrOn
    if (m_active) {
      powerModelPort->reportState(m_offStateId);
      m_active = false;
    }
  }
}

void Adc12::process() {
  if (pwrOn.read()) {
    // Default trigger
    next_trigger(samplingClock.default_event() | pwrOn.negedge_event());

    if (m_regs.testBitMask(OFS_ADC12CTL0, ADC12ON) &&
        m_regs.testBitMask(OFS_ADC12CTL0, ADC12ENC) && pwrOn.read()) {
      // ADC is on
      if (!m_active) {
        m_active = true;
        powerModelPort->reportState(m_onStateId);
      }

    } else {
      // ADC is off, wait for bus transaction
      if (m_active) {
        powerModelPort->reportState(m_offStateId);
        m_active = false;
      }
      next_trigger(modeEvent | pwrOn.negedge_event());
      irq.write(false);
      return;
    }

    // Clear irq flag if IV register cleared.
    if (m_regs.read(OFS_ADC12IV) == 0) {
      irq.write(false);
    }

    // Save sample to output value register
    int resolution = -1;
    switch (m_regs.read(OFS_ADC12CTL2) & ADC12RES) {
      case ADC12RES__8BIT:
        resolution = 1u << 8;
        break;
      case ADC12RES__10BIT:
        resolution = 1u << 10;
        break;
      case ADC12RES__12BIT:
        resolution = 1u << 12;
        break;
      default:
        spdlog::error("{}: Invalid ADC12RES (ADC12CTL2= 0x{:04x})",
                      this->name(), m_regs.read(OFS_ADC12CTL2));
        SC_REPORT_FATAL(this->name(), "Invalid ADC12RES");
        break;
    }
    uint16_t sample = static_cast<uint16_t>(
        static_cast<double>(resolution) *
        (vcc.read() / 4.0));  // Assumes 2v ref and meas vcc/2
    m_regs.write(OFS_ADC12MEM0, sample);
    powerModelPort->reportEvent(m_sampleEventId);

    // Window comparator low interrupt
    if (m_regs.testBitMask(OFS_ADC12IER2, ADC12LOIE) &&
        sample < m_regs.read(OFS_ADC12LO)) {
      // Sample smaller than low threshold
      m_regs.setBitMask(OFS_ADC12IV, ADC12IV__ADC12LOIFG);
      irq.write(true);
      next_trigger(m_writeEvent | samplingClock.default_event() |
                   pwrOn.negedge_event());
    }

    // Window comparator high interrupt
    if (m_regs.testBitMask(OFS_ADC12IER2, ADC12HIIE) &&
        (sample > m_regs.read(OFS_ADC12HI))) {
      // Sample exceeds high threshold
      m_regs.setBitMask(OFS_ADC12IV, ADC12IV__ADC12HIIFG);
      irq.write(true);
      next_trigger(m_writeEvent | samplingClock.default_event() |
                   pwrOn.negedge_event());
    }
  } else if (pwrOn.read() == false) {
    // Wait for power
    next_trigger(pwrOn.posedge_event());
  }
}

void Adc12::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  // Access register file
  BusTarget::b_transport(trans, delay);
  auto addr = trans.get_address();

  // Handle ADC events
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    switch (addr) {
      case OFS_ADC12CTL0:
        modeEvent.notify(delay);
        samplingClockUpdateEvent.notify(delay);
        break;
      case OFS_ADC12CTL1:
        samplingClockUpdateEvent.notify(delay);
        break;
      case OFS_ADC12CTL2:
        samplingClockUpdateEvent.notify(delay);
        break;
    }
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    switch (addr) {
      case OFS_ADC12IV:
        modeEvent.notify(delay);
        m_regs.write(OFS_ADC12IV, 0);
        break;
    }
  }
}

void Adc12::updateClkSource() {
  // Mux
  auto sel = m_regs.read(OFS_ADC12CTL1) & ADC12SSEL;

  switch (sel) {       // Clock source select
    case ADC12SSEL_0:  // MODCLK
      clkMuxSelect.write(MODCLK_SEL);
      break;
    case ADC12SSEL_1:  // ACLK
      clkMuxSelect.write(ACLK_SEL);
      break;
    case ADC12SSEL_2:  // MCLK
      clkMuxSelect.write(MCLK_SEL);
      break;
    case ADC12SSEL_3:  // SMCLK
      clkMuxSelect.write(MCLK_SEL);
      break;
  }

  int div = 1;
  // Pre-divider
  switch (m_regs.read(OFS_ADC12CTL1) & ADC12PDIV) {
    case ADC12PDIV_0:
      div = 1;
      break;
    case ADC12PDIV_1:
      div = 4;
      break;
    case ADC12PDIV_2:
      div = 32;
      break;
    case ADC12PDIV_3:
      div = 64;
      break;
  }

  // Divider
  div *= (1 + ((m_regs.read(OFS_ADC12CTL1) & ADC12DIV) >> 5));

  // Sample & hold time
  int sampleTime = 0;
  switch (m_regs.read(OFS_ADC12CTL0) & ADC12SHT0) {
    case ADC12SHT0_0:
      sampleTime = 4;
      break;
    case ADC12SHT0_1:
      sampleTime = 8;
      break;
    case ADC12SHT0_2:
      sampleTime = 16;
      break;
    case ADC12SHT0_3:
      sampleTime = 32;
      break;
    case ADC12SHT0_4:
      sampleTime = 64;
      break;
    case ADC12SHT0_5:
      sampleTime = 96;
      break;
    case ADC12SHT0_6:
      sampleTime = 128;
      break;
    case ADC12SHT0_7:
      sampleTime = 256;
      break;
    case ADC12SHT0_8:
      sampleTime = 384;
      break;
    case ADC12SHT0_9:
      sampleTime = 512;
      break;
    default:
      spdlog::error("{}: Invalid ADC12SHT0 0x{:04x}", this->name(),
                    (m_regs.read(OFS_ADC12CTL1) & ADC12SHT0) >> 8);
      SC_REPORT_FATAL(this->name(), "Invalid ADC12SHT0");
      break;
  }

  // Conversion time
  int conversionTime = -1;
  switch (m_regs.read(OFS_ADC12CTL2) & ADC12RES) {
    case ADC12RES__8BIT:
      conversionTime = 10;
      break;
    case ADC12RES__10BIT:
      conversionTime = 12;
      break;
    case ADC12RES__12BIT:
      conversionTime = 14;
      break;
    default:
      spdlog::error("{}: Invalid ADC12RES", this->name());
      SC_REPORT_FATAL(this->name(), "Invalid ADC12RES");
  }
  div *= (sampleTime + conversionTime);
  clkDivAmount.write(div);
}
