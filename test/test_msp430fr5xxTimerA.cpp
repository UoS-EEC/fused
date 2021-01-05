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
#include "ps/PowerModelChannel.hpp"
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
  sc_signal<bool> dmaTrigger{"dmaTrigger"};

  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel smclk_sig{"smclk_sig", sc_time(1, SC_US)};
  ClockSourceChannel aclk_sig{"aclk_sig", sc_time(1, SC_US)};
  ClockSourceChannel clk{"clk", sc_time(1, SC_NS)};
  PowerModelChannel powerModelChannel{
      "powerModelChannel", "/tmp/testPowerModelChannel.csv",
      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.tSocket.bind(iSocket);
    m_dut.smclk.bind(smclk_sig);
    m_dut.aclk.bind(aclk_sig);
    m_dut.irq.bind(irq);
    m_dut.ira.bind(ira);
    m_dut.dmaTrigger.bind(dmaTrigger);
    m_dut.systemClk.bind(clk);
    m_dut.powerModelPort.bind(powerModelChannel);
  }

  TimerA m_dut{"dut", 0};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // TEST -- Up mode: a basic interrupt

    // Mode control + interrupt enable + select ACLK
    write16(OFS_TA1CTL, MC_1 | TAIE | TASSEL_1);

    // Target value
    write16(OFS_TA1CCR0, 10 - 1);

    // Check
    wait(sc_time(10, SC_US));
    sc_assert(test.irq.read() == true);
    test.ira.write(true);
    wait(sc_time(2, SC_US));
    sc_assert(test.irq.read() == false);
    test.ira.write(false);

    // TEST -- Up mode: dma trigger
    test.pwrGood.write(false);
    wait(sc_time(2, SC_US));
    test.pwrGood.write(true);
    wait(sc_time(1, SC_US));
    write16(OFS_TA1CTL, MC_1 | TASSEL_1);  // Disable irq
    write16(OFS_TA1CCR0, 10 - 1);

    wait(sc_time(10, SC_US));
    sc_assert(test.dmaTrigger.read() == true);
    wait(sc_time(2, SC_US));
    sc_assert(test.dmaTrigger.read() == false);

    sc_stop();
  }

  void write16(const uint32_t addr, const uint32_t val, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[2];
    trans.set_data_ptr(data);
    trans.set_data_length(2);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);

    Utility::unpackBytes(data, Utility::htots(val), 2);
    test.iSocket->b_transport(trans, delay);
    if (doWait) {
      wait(delay);
    }
  }

  uint32_t read16(const uint32_t addr, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[2];
    trans.set_data_ptr(data);
    trans.set_data_length(2);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);
    test.iSocket->b_transport(trans, delay);

    if (doWait) {
      wait(delay);
    }
    return Utility::ttohs(Utility::packBytes(data, 2));
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
