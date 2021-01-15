/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <stdexcept>
#include <string>
#include <systemc>
#include "libs/make_unique.hpp"
#include "ps/ConstantCurrentState.hpp"
#include "ps/ConstantEnergyEvent.hpp"
#include "ps/PowerModelChannel.hpp"
#include "ps/PowerModelChannelIf.hpp"

using namespace sc_core;

SC_MODULE(dut) {
 public:
  PowerModelEventInPort inport{"inport"};
  PowerModelEventOutPort outport{"outport"};
  PowerModelChannel ch{"ch", "/tmp", sc_time(1, SC_US)};
  SC_CTOR(dut) {
    inport(ch);
    outport(ch);
  }
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) {
    registerEvents();
    registerStates();
    SC_THREAD(runtests);
  }

  void registerEvents() {
    spdlog::info("------ TEST: register some events");
    eid1 = test.outport->registerEvent(
        "module0", std::make_unique<ConstantEnergyEvent>("event1", 1.0e-12));
    sc_assert(eid1 == 0);
    eid2 = test.outport->registerEvent(
        "module0", std::make_unique<ConstantEnergyEvent>("event2", 2.0e-12));
    sc_assert(eid2 == 1);

    spdlog::info(
        "------ TEST: registering the same event twice throws exception");
    auto success = false;
    try {
      test.outport->registerEvent(
          "module0", std::make_unique<ConstantEnergyEvent>("event1", 1.0e-12));
    } catch (std::invalid_argument &e) {
      success = true;
    }
    sc_assert(success);
  }

  void registerStates() {
    spdlog::info("------ TEST: register some states");
    sid1 = test.outport->registerState(
        "module0", std::make_unique<ConstantCurrentState>("off", 0.0));
    sc_assert(sid1 == 0);
    sid2 = test.outport->registerState(
        "module0", std::make_unique<ConstantCurrentState>("on", 1.0e-6));
    sc_assert(sid2 == 1);
    sid3 = test.outport->registerState(
        "module1", std::make_unique<ConstantCurrentState>("off", 0.0));
    sc_assert(sid3 == 2);
    sid4 = test.outport->registerState(
        "module1", std::make_unique<ConstantCurrentState>("on", 2.0e-6));
    sc_assert(sid4 == 3);

    spdlog::info(
        "------ TEST: registering the same state for the same module twice "
        "throws exception");
    auto success = false;
    try {
      sid1 = test.outport->registerState(
          "module0", std::make_unique<ConstantCurrentState>("on", 1.0e-6));
    } catch (std::invalid_argument &e) {
      success = true;
    }
    sc_assert(success);
  }

  void runtests() {
    spdlog::info(
        "------ TEST: Registering an event after simulation start causes "
        "exception");
    try {
      test.outport->registerEvent(
          "module0", std::make_unique<ConstantEnergyEvent>("event1", 1.0e-12));
      sc_assert(false);  // Fail
    } catch (std::runtime_error &e) {
      // Success
    }

    spdlog::info("------ TEST: Event counts add up");
    test.outport->reportEvent(eid1, 1);
    test.outport->reportEvent(eid1, 1);
    test.outport->reportEvent(eid1, 1);
    sc_assert(test.inport->popEventCount(eid1) == 3);

    spdlog::info("------ TEST: Event counts reset after pop");
    sc_assert(test.inport->popEventCount(eid1) == 0);

    spdlog::info("------ TEST: Single-channel event energy adds up");
    test.outport->reportEvent(eid1, 1);
    test.outport->reportEvent(eid1, 1);
    test.outport->reportEvent(eid1, 3);
    sc_assert(test.inport->popEventEnergy(eid1) == 5 * 1.0e-12);

    spdlog::info("------ TEST: Single-channel event energy resets after pop");
    sc_assert(test.inport->popEventEnergy(eid1) == 0.0);

    spdlog::info("------ TEST: Multi-channel event energy adds up");
    test.outport->reportEvent(eid1, 1);
    test.outport->reportEvent(eid2, 1);
    sc_assert(test.inport->popDynamicEnergy() == 1 * 1.0e-12 + 1 * 2.0e-12);

    spdlog::info("------ TEST: Multi-channel event energy resets after pop");
    sc_assert(test.inport->popDynamicEnergy() == 0.0);

    spdlog::info("------ TEST: Static current sums up");
    test.outport->reportState(sid2);
    test.outport->reportState(sid4);
    sc_assert(test.inport->getStaticCurrent() == 3.0e-6);
    test.outport->reportState(sid1);
    test.outport->reportState(sid3);
    sc_assert(test.inport->getStaticCurrent() == 0.0);

    sc_stop();
  }

  int eid1;
  int eid2;
  int sid1;
  int sid2;
  int sid3;
  int sid4;

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  tester t("tester");

  sc_start();
  return false;
}
