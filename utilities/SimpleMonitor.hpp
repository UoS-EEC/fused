/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <mcu/BusTarget.hpp>
#include <systemc>
#include <vector>
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"

extern "C" {
#include "include/fused.h"
}

/** SC Module SimpleMonitor
 * SimpleMonitor implements a single register to control simulation and reports
 * written values to the console.
 */
class SimpleMonitor : public BusTarget {
  SC_HAS_PROCESS(SimpleMonitor);

 public:
  /**
   * Constructor
   */
  SimpleMonitor(const sc_core::sc_module_name nm, const sc_core::sc_time delay)
      : BusTarget(nm, SIMPLE_MONITOR_BASE, SIMPLE_MONITOR_BASE + 4, delay) {
    SC_METHOD(process);
    sensitive << m_writeEvent;
    m_regs.addRegister(0, 0);
  }

 private:
  /*------ Private variables ------*/

  /* ------ Private functions ------*/
  void process() {
    auto reg = m_regs.read(0);  // Get written value

    spdlog::info(
        "{}: {:010d} ns 0x{:08x}", this->name(),
        static_cast<unsigned long>(sc_core::sc_time_stamp().to_seconds() * 1e9),
        reg);

    switch (reg) {
      case SIMPLE_MONITOR_KILL_SIM:  // Stop simulation
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
      case SIMPLE_MONITOR_START_EVENT_LOG:  // Trigger event log
        spdlog::info("{}: Signaling EventLog to start recording.",
                     this->name());
        EventLog::getInstance().startLogging(sc_core::SC_ZERO_TIME);
        break;
      case SIMPLE_MONITOR_INDICATE_BEGIN:
        break;
      case SIMPLE_MONITOR_INDICATE_END:
        break;
    }
  }

  virtual void reset() override { /* Do nothing */
  }
};
