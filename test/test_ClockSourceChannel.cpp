/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"

using namespace sc_core;

SC_MODULE(dut) {
 public:
  SC_CTOR(dut) {
    SC_METHOD(countClockEdges);
    sensitive << m_dut;
    dont_initialize();

    SC_METHOD(countPeriodChanges);
    sensitive << m_dut.periodChangedEvent();
    dont_initialize();
  }

  // Count clock edges
  void countClockEdges() { m_edgeCount++; }
  void countPeriodChanges() { m_periodChangeCount++; }

  // Reset counters
  void reset() {
    m_edgeCount = 0;
    m_periodChangeCount = 0;
  }

  ClockSourceChannel m_dut{"clockSource"};

  int m_edgeCount{0};
  int m_periodChangeCount{0};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    sc_time basePeriod = sc_time(1, SC_US);

    // TEST 1 Start & stop clock
    test.reset();
    test.m_dut.setPeriod(basePeriod);
    wait(3 * basePeriod);
    sc_assert(test.m_edgeCount == 3);
    sc_assert(test.m_periodChangeCount == 1);

    // TEST 2 Reset
    test.reset();
    test.m_edgeCount = 0;
    test.m_dut.setPeriod(basePeriod);
    wait(basePeriod);

    test.m_dut.reset();
    test.reset();
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.getPeriod() == SC_ZERO_TIME);

    wait(3 * basePeriod);
    sc_assert(test.m_edgeCount == 0);

    // TEST 3 Change period
    test.reset();
    test.m_dut.reset();
    test.m_dut.setPeriod(basePeriod);
    wait(2.5 * basePeriod);
    sc_assert(test.m_edgeCount == 2);
    sc_assert(test.m_periodChangeCount == 1);
    test.m_dut.setPeriod(2 * basePeriod);
    sc_assert(test.m_dut.getPeriod() == 2 * basePeriod);
    wait(2 * basePeriod);
    sc_assert(test.m_edgeCount == 2 + 1);
    sc_assert(test.m_periodChangeCount == 2);

    sc_stop();
  }

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  tester t("tester");
  sc_start();
  return false;
}
