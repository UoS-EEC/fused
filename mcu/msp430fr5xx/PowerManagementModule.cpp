/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <systemc>
#include <tuple>
#include <vector>
#include "libs/strtk.hpp"
#include "mcu/msp430fr5xx/PowerManagementModule.hpp"
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

#define PMM_SIZE (OFS_PM5CTL0 + 2)
#define OFS_PMMCTL1 (OFS_PMMCTL0 + 2)

using namespace sc_core;

SC_HAS_PROCESS(PowerManagementModule);

PowerManagementModule::PowerManagementModule(sc_module_name name)
    : PowerManagementModule(name, PMM_BASE, PMM_BASE + PMM_SIZE - 1) {}

PowerManagementModule::PowerManagementModule(sc_module_name name,
                                             unsigned startAddress,
                                             unsigned endAddress)
    : BusTarget(name, startAddress, endAddress),
      m_bootCurrentState(
          std::make_shared<BootCurrentState>("bootCurrentState")) {
  m_vOn = Config::get().getDouble("PMMOn");
  m_vOff = Config::get().getDouble("PMMOff");
  m_vMax = Config::get().getDouble("VMAX");

  // Note: correct values are written during reset()
  m_regs.addRegister(OFS_PMMCTL0, 0x9640, RegisterFile::AccessMode::READ_WRITE);
  m_regs.addRegister(OFS_PMMCTL1, 0x9600, RegisterFile::AccessMode::READ_WRITE);
  m_regs.addRegister(OFS_PMMIFG, 0, RegisterFile::AccessMode::READ_WRITE);
  m_regs.addRegister(OFS_PM5CTL0, 0x0001, RegisterFile::AccessMode::READ_WRITE);

  reset();

  // Load boot current trace
  Utility::assertFileExists(Config::get().getString("BootTracePath"));
  std::ifstream file(Config::get().getString("BootTracePath"));
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string content = buffer.str();

  strtk::token_grid grid(content, content.size(), ",");
  for (std::size_t i = 0; i < grid.row_count(); ++i) {
    m_bootCurrentTrace.push_back(1E-3 * grid.row(i).get<double>(1));
  }
  m_bootCurrentTimeResolution =
      grid.row(1).get<double>(0) - grid.row(0).get<double>(0);
}

void PowerManagementModule::end_of_elaboration() {
  powerModelPort->registerState(this->name(), m_bootCurrentState);

  SC_THREAD(process);
}

void PowerManagementModule::reset(void) {
  // Reset registers to their default values
  m_regs.reset();
  m_locked = true;
  m_isOn = false;
}

void PowerManagementModule::process(void) {
  // staticPower.write(0.0);

  wait(SC_ZERO_TIME);

  while (true) {
    // TODO: Handle software POR & BOR

    double crntVcc = vcc.read();
    if (m_isOn) {
      // CPU is on, wait for supply to drop
      if (crntVcc < m_vOff) {
        m_isOn = false;
        pwrGood.write(m_isOn);
      }
    } else {
      // CPU is off, wait for supply to recover
      if (crntVcc > m_vOn) {
        // Replay boot-current trace
        for (const auto &v : m_bootCurrentTrace) {
          m_bootCurrentState->setCurrent(v);
          wait(sc_time::from_seconds(m_bootCurrentTimeResolution));
        }
        m_bootCurrentState->setCurrent(0.0);

        m_isOn = true;
        pwrGood.write(m_isOn);

        irq.write(true);  // Power-On Reset
      }
    }

    if (ira.read()) {
      irq.write(false);
    }

    if (crntVcc > m_vMax) {
      SC_REPORT_WARNING(this->name(), "Vcc exceeds vMax!");
    }

    wait(vcc.value_changed_event() | ira.default_event());
  }
}

void PowerManagementModule::b_transport(tlm::tlm_generic_payload &trans,
                                        sc_time &delay) {
  static const unsigned PW_MASK = 0xff00;
  auto len = trans.get_data_length();
  auto addr = trans.get_address();
  uint16_t *src = reinterpret_cast<uint16_t *>(trans.get_data_ptr());

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    switch (addr) {
      case OFS_PMMCTL0:
        m_locked = ((*src & PW_MASK) == PMMPW);
        break;
      case OFS_PMMCTL0_H:
        m_locked = (*src == PMMPW);
        break;
      case OFS_PM5CTL0:
        if (*src == 0) {
          m_regs.write(addr, 0);
        } else {
          // Ignore set
        }
        break;
      default:
        if (len == 2) {
          m_regs.write(addr, *src);
        } else if (len == 1) {
          m_regs.writeByte(addr, static_cast<uint8_t>(*src));
        }
    }

    m_writeEvent.notify(delay + systemClk->getPeriod());

  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    switch (addr) {
      default:
        if (len == 2) {
          *src = m_regs.read(addr);
        } else if (len == 1) {
          *src = m_regs.readByte(addr);
        }
        break;
    }

    m_readEvent.notify(delay + systemClk->getPeriod());
  } else {
    SC_REPORT_FATAL(this->name(), "Payload command not supported.");
  }

  delay += systemClk->getPeriod();
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}
