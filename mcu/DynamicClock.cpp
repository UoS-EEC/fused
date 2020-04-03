/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "mcu/DynamicClock.hpp"

using namespace sc_core;

DynamicClock::DynamicClock(sc_module_name name, double period, sc_time_unit tu)
    : sc_module(name), _period(period, tu) {
  SC_HAS_PROCESS(DynamicClock);
  SC_THREAD(process);
}

void DynamicClock::setPeriod(sc_time newPeriod) {
  // Cancel queued edge
  nextEdgeEvent.cancel();

  // Queue next edge
  nextEdgeEvent.notify(newPeriod / 2);
  _period = newPeriod;
}

sc_time DynamicClock::period(void) { return _period; }

void DynamicClock::process(void) {
  out.write(false);
  wait(SC_ZERO_TIME);
  nextEdgeEvent.notify(_period / 2);

  while (true) {
    wait(nextEdgeEvent);
    out.write(!out.read());
    nextEdgeEvent.notify(_period / 2);
  }
}
