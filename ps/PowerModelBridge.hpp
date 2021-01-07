/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
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
    const double timestep =
        (sc_core::sc_time_stamp() - m_lastReadTime).to_seconds();

    // Current = static_current + E/(v*ts)
    const double i =
        powerModelPort->getStaticCurrent() +
        powerModelPort->popDynamicEnergy() / (v_in.read() * timestep);
    i_out.write(i);
    m_lastReadTime = sc_core::sc_time_stamp();
  }
  sc_core::sc_time m_lastReadTime{sc_core::SC_ZERO_TIME};
};
