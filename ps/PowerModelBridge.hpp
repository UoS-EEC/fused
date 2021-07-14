/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "ps/PowerModelChannelIf.hpp"
#include <spdlog/spdlog.h>
#include <systemc-ams>
#include <systemc>

/**
 * @brief PowerModelBridge bridge between PowerModelChannel and sc_signals
 *
 * Updates i_out every time vcc is updated.
 */
SC_MODULE(PowerModelBridge) {
  sc_core::sc_out<double> i_out{"i_out"};
  sc_core::sc_in<double> v_in{"v_in"};
  PowerModelEventInPort powerModelPort{"PowerModelPort"};

  PowerModelBridge(const sc_core::sc_module_name name,
                   const sc_core::sc_time timestep)
      : sc_core::sc_module(name), m_timestep(timestep) {
    SC_HAS_PROCESS(PowerModelBridge);
    SC_THREAD(process);
    SC_METHOD(updateVcc);
    sensitive << v_in;
  }

  void updateVcc() { powerModelPort->setSupplyVoltage(v_in.read()); }

  void process() {
    i_out.write(0.0);
    while (1) {
      wait(m_timestep);
      if (v_in.read() <= 0.0) {
        volatile const double dynamicCurrent =
            powerModelPort->popDynamicEnergy() /
            (v_in.read() * m_timestep.to_seconds());
        volatile const double i =
            powerModelPort->getStaticCurrent() + dynamicCurrent;
        i_out.write(0.0);
      } else {
        // Dynamic current = E/(v*ts)
        const double dynamicCurrent = powerModelPort->popDynamicEnergy() /
                                      (v_in.read() * m_timestep.to_seconds());

        const double i = powerModelPort->getStaticCurrent() + dynamicCurrent;
        i_out.write(i);

        // spdlog::info(FMT_STRING(
        // "{:s}: {:010d} us delta {:.1f} us static {:.6f} mA dynamic {:.6f}
        // mA"),
        //             this->name(),
        //             static_cast<long long unsigned int>(
        //                 1e6 * sc_core::sc_time_stamp().to_seconds()),
        //             1e6 * timestep, 1e3 * powerModelPort->getStaticCurrent(),
        //             1e3 * dynamicCurrent);
      }
    }
  }

  // sc_core::sc_time m_lastReadTime{sc_core::SC_ZERO_TIME};
  const sc_core::sc_time m_timestep;
};
