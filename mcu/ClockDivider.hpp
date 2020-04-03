/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>
#include <systemc>
#include "mcu/ClockSourceIf.hpp"
#include "utilities/Config.hpp"

SC_MODULE(ClockDivider) {
 public:
  sc_core::sc_port<ClockSourceConsumerIf> inClk{"inClk"};
  sc_core::sc_port<ClockSourceDriverIf> outClk{"outClk"};
  sc_core::sc_in<int> divIn{"divIn"};

  SC_CTOR(ClockDivider) {}

  void end_of_elaboration() {
    SC_METHOD(process);
    sensitive << inClk->periodChangedEvent() << divIn;
  }

 private:
  void process() {
    sc_assert(divIn->read() > 0);  // Don't divide by 0
    outClk->setPeriod(inClk->getPeriod() * divIn->read());
  }
};
