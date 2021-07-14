/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "include/peripheral-defines.h"
#include "utilities/Config.hpp"
#include <mcu/BusTarget.hpp>
#include <spdlog/spdlog.h>
#include <stdint.h>
#include <systemc>
#include <vector>

/** SC Module SimpleMonitor
 * SimpleMonitor implements a single register to control simulation and reports
 * written values to the console.
 *
 * Implements a simulation control register, and a character output register.
 *
 * The simulation control register can be used to signal events, errors etc.,
 * and to stop simulation.
 *
 * The character output register can be used to display strings from the target.
 * Write the characters one by one. When a 0 is written, the output gets printed
 * to Fused's output, along with a time stamp.
 */
class SimpleMonitor : public BusTarget {
  SC_HAS_PROCESS(SimpleMonitor);

  struct RegisterAddress {

    // Simulation control register
    static const unsigned SIMCTRL = 0x00;

    // Character output register.
    static const unsigned PRINT = 0x04;
  };

public:
  /**
   * Constructor
   */
  SimpleMonitor(const sc_core::sc_module_name nm, const unsigned startAddress)
      : BusTarget(nm, startAddress, startAddress + 2 * 4 - 1) {
    SC_METHOD(process);
    sensitive << m_writeEvent;
    m_regs.addRegister(RegisterAddress::SIMCTRL,
                       /*resetValue=*/0xAAAAAAAA);
    m_regs.addRegister(RegisterAddress::PRINT,
                       /*resetValue=*/0xAAAAAAAA);
  }

private:
  /*------ Private variables ------*/
  std::string m_console; //! String to store console output

  /* ------ Private functions ------*/
  void process() {
    const auto ctrl = m_regs.read(0); // Get written value

    if (ctrl != 0xAAAAAAAA) {
      spdlog::info("{}: {:010d} ns 0x{:08x}", this->name(),
                   static_cast<unsigned long>(
                       sc_core::sc_time_stamp().to_seconds() * 1e9),
                   ctrl);

      switch (ctrl) {
      case SIMPLE_MONITOR_KILL_SIM: // Stop simulation
        spdlog::info("{}: Stopping simulation", this->name());
        sc_core::sc_stop();
        break;
      case SIMPLE_MONITOR_SW_ERROR:
        SC_REPORT_FATAL(this->name(), "SW_ERROR: CPU reported software error");
        break;
      case SIMPLE_MONITOR_TEST_FAIL:
        SC_REPORT_FATAL(this->name(),
                        "SW_TEST_FAIL: CPU reported software test fail");
        break;
      case SIMPLE_MONITOR_INDICATE_BEGIN:
        break;
      case SIMPLE_MONITOR_INDICATE_END:
        break;
      }
    }

    const auto reg2 = m_regs.read(4);
    if (reg2 == 0) {
      spdlog::info("{}: Console print @{:.0f} ns\n\t{}\n", this->name(),
                   sc_core::sc_time_stamp().to_seconds() * 1e9, m_console);
      m_console.clear();
    } else if (reg2 != 0xAAAAAAAA) {
      m_console += static_cast<uint8_t>(reg2);
    }
    m_regs.reset();
  }

  virtual void reset() override { m_regs.reset(); }
};
