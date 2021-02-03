/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <spdlog/spdlog.h>
#include <systemc-ams>
#include <systemc>
#include "ps/PowerModelChannelIf.hpp"

/**
 * @brief PowerModelBridge bridge between PowerModelChannel and sc_signals
 *
 * Updates i_out every time vcc is updated.
 */
SC_MODULE(PowerModelBridge) {
  sc_core::sc_out<double> i_out{"i_out"};
  sc_core::sc_in<double> v_in{"v_in"};
  PowerModelEventInPort powerModelPort{"PowerModelPort"};

  SC_CTOR(PowerModelBridge) {
    SC_METHOD(process);
    sensitive << v_in;
    dont_initialize();
  }

  void process() {
    if (v_in.read() <= 0.0) {
      i_out.write(0.0);
      return;
    }
    const double timestep =
        (sc_core::sc_time_stamp() - m_lastReadTime).to_seconds();
    m_lastReadTime = sc_core::sc_time_stamp();

    // Dynamic current = E/(v*ts)
    const double dynamicCurrent =
        powerModelPort->popDynamicEnergy() / (v_in.read() * timestep);

    const double i = powerModelPort->getStaticCurrent() + dynamicCurrent;
    i_out.write(i);

    // spdlog::info(FMT_STRING(
    // "{:s}: {:010d} us delta {:.1f} us static {:.6f} mA dynamic {:.6f} mA"),
    //             this->name(),
    //             static_cast<long long unsigned int>(
    //                 1e6 * sc_core::sc_time_stamp().to_seconds()),
    //             1e6 * timestep, 1e3 * powerModelPort->getStaticCurrent(),
    //             1e3 * dynamicCurrent);
  }

  sc_core::sc_time m_lastReadTime{sc_core::SC_ZERO_TIME};
};
