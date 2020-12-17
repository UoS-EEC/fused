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
#include "ps/ConstantEnergyEvent.hpp"
#include "ps/PowerModelEventChannel.hpp"
#include "ps/PowerModelEventChannelIf.hpp"

using namespace sc_core;

SC_MODULE(dut) {
 public:
  PowerModelEventInPort inport{"inport"};
  PowerModelEventOutPort outport{"outport"};
  PowerModelEventChannel ch;
  SC_CTOR(dut) {
    inport(ch);
    outport(ch);
  }
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    spdlog::info("------ TEST: register some events");
    sc_assert(test.inport->size() == 0);
    auto eid1 =
        test.outport->registerEvent(std::unique_ptr<ConstantEnergyEvent>(
            new ConstantEnergyEvent("event1", 1.0e-12)));
    sc_assert(test.inport->size() == 1);
    auto eid2 =
        test.outport->registerEvent(std::unique_ptr<ConstantEnergyEvent>(
            new ConstantEnergyEvent("event2", 2.0e-12)));
    sc_assert(test.inport->size() == 2);

    spdlog::info(
        "------ TEST: registering the same name twice throws exception");
    auto success = false;
    try {
      test.outport->registerEvent(std::unique_ptr<ConstantEnergyEvent>(
          new ConstantEnergyEvent("event1", 1.0e-12)));
    } catch (std::invalid_argument &e) {
      success = true;
    }
    sc_assert(success);

    spdlog::info("------ TEST: Event counts add up");
    test.outport->write(eid1, 1);
    test.outport->write(eid1, 1);
    test.outport->write(eid1, 1);
    sc_assert(test.inport->pop(eid1) == 3);

    spdlog::info("------ TEST: Event counts reset after pop");
    sc_assert(test.inport->pop(eid1) == 0);

    spdlog::info("------ TEST: Single-channel event energy adds up");
    test.outport->write(eid1, 1);
    test.outport->write(eid1, 1);
    test.outport->write(eid1, 3);
    sc_assert(test.inport->popEnergy(eid1, 0.0) == 5 * 1.0e-12);

    spdlog::info("------ TEST: Single-channel event energy resets after pop");
    sc_assert(test.inport->popEnergy(eid1) == 0.0);

    spdlog::info("------ TEST: Multi-channel event energy adds up");
    test.outport->write(eid1, 1);
    test.outport->write(eid2, 1);
    sc_assert(test.inport->popEnergy(0.0) == 1 * 1.0e-12 + 1 * 2.0e-12);

    spdlog::info("------ TEST: Multi-channel event energy resets after pop");
    sc_assert(test.inport->popEnergy(eid1) == 0.0);

    sc_stop();
  }

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  tester t("tester");
  sc_start();
  return false;
}
