/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <systemc>
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"

using namespace sc_core;

template <unsigned N>
SC_MODULE(ClockMux) {
 public:
  std::array<sc_port<ClockSourceConsumerIf>, N> inClk;  //! Input clock
  sc_port<ClockSourceDriverIf> outClk{"outClk"};        //! Output clock
  sc_in<int> sel;  //! Clock source selector

  SC_CTOR(ClockMux){};

  void end_of_elaboration() {
    SC_METHOD(process);
    sensitive << sel;
    for (const auto &c : inClk) {
      sensitive << c->periodChangedEvent();
    }
  }

 private:
  void process() {
    assert(sel->read() <= N);
    outClk->setPeriod(inClk[sel->read()]->getPeriod());
  }
};
