/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <systemc>
#include "utilities/Config.hpp"

/** SC_MODULE IoSimulationStopper
 * Stops simulation after N posedges of a boolean signal (e.g. IO pin)
 * connected to in
 */
SC_MODULE(IoSimulationStopper) {
  sc_core::sc_in_resolved in{"in"};

  /**
   * Constructor
   */
  SC_CTOR(IoSimulationStopper) {
    m_target = Config::get().getUint("IoSimulationStopperTarget");
    SC_THREAD(process);
  }

 private:
  /*------ Private variables ------*/
  unsigned int m_target;

  /* ------ Private functions ------*/
  void process() {
    wait(sc_core::SC_ZERO_TIME);
    static unsigned int cnt = 0;
    while (true) {
      wait(in.posedge_event());
      cnt++;
      if (cnt >= m_target) {
        spdlog::info(
            "{:s}: Simulation stopping at {:010.0f} ns after IO count: {:d}",
            this->name(), sc_core::sc_time_stamp().to_seconds() * 1e9,
            m_target);
        wait(sc_core::sc_time(10, sc_core::SC_US));
        sc_core::sc_stop();
      }
    }
  }
};
