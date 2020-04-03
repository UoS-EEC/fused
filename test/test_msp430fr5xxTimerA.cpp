/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/msp430fr5xx/ClockSystem.hpp"
#include "mcu/msp430fr5xx/TimerA.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

SC_MODULE(dut) {
 public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood"};
  sc_signal<bool> irq{"irq"};
  sc_signal<bool> ira{"ira"};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel smclk_sig{"smclk_sig", sc_time(1, SC_US)};
  ClockSourceChannel aclk_sig{"aclk_sig", sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.tSocket.bind(iSocket);
    m_dut.smclk.bind(smclk_sig);
    m_dut.aclk.bind(aclk_sig);
    m_dut.irq.bind(irq);
    m_dut.ira.bind(ira);
  }

  TimerA m_dut{"dut", 0, sc_time(1, SC_NS)};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    unsigned char data[] = {255, 255};
    sc_time delay;
    tlm::tlm_generic_payload trans;
    trans.set_data_ptr(data);
    trans.set_data_length(2);

    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // TEST -- Up mode: a basic interrupt

    // Mode control + interrupt enable + select ACLK
    Utility::unpackBytes(data, Utility::htots(MC_1 | TAIE | TASSEL_1), 2);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(OFS_TA1CTL);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

    // Target value
    Utility::unpackBytes(data, Utility::htots(10 - 1), 2);
    trans.set_address(OFS_TA1CCR0);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);
    wait(delay);

    wait(sc_time(10, SC_US));
    sc_assert(test.irq.read() == true);
    test.ira.write(true);
    wait(sc_time(2, SC_US));
    sc_assert(test.irq.read() == false);
    test.ira.write(false);

    sc_stop();
  }

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  // Set up paths
  // Parse CLI arguments & config file
  auto &config = Config::get();
  config.parseFile();

  // Instantiate and hook up event log to dummy signals
  sc_signal<double> elogStaticConsumption{"elogStaticConsumption"};
  DynamicEnergyChannel elogDynamicConsumption("elogDynamicConsumption");

  auto &elog = EventLog::getInstance();
  elog.staticPower.bind(elogStaticConsumption);
  elog.dynamicEnergy.bind(elogDynamicConsumption);

  tester t("tester");
  sc_start();
  return false;
}
