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
#include "mcu/cortex-m0/SysTick.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
 public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood", false};
  sc_signal<bool> irq{"irq"};
  sc_signal<int> returning_exception{"returning_exception", -1};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel clk{"clk", sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.tSocket.bind(iSocket);
    m_dut.clk.bind(clk);
    m_dut.irq.bind(irq);
    m_dut.returning_exception.bind(returning_exception);
  }

  SysTick m_dut{"dut", sc_time(1, SC_NS)};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // TEST -- CSR reset value
    sc_assert(read32(OFS_SYST_CSR) == 0x4);

    // TEST -- TENMS register
    sc_assert(
        read32(OFS_SYST_CALIB, false) ==
        (SYST_CALIB_NOREF |
         (static_cast<int>(sc_time(10, SC_MS) / test.clk.getPeriod()) - 1)));

    // TEST -- Set up, but don't enable
    test.m_dut.reset();
    write32(OFS_SYST_RVR, 100 - 1, false);  // Reload value for 100 ticks
    write32(OFS_SYST_CVR, 100 - 1, false);  // Value should be ignored
    sc_assert(read32(OFS_SYST_CVR, false) == 0);
    wait(sc_time(200, SC_US));
    sc_assert(read32(OFS_SYST_CVR) == 0);

    // TEST -- Enable and check count
    write32(OFS_SYST_CSR, SYST_CSR_ENABLE, false);
    wait(sc_time(10, SC_US));
    sc_assert(read32(OFS_SYST_CVR) == 99 - 10);

    // TEST -- Check wrap & COUNTFLAG
    wait(sc_time(99 - 10 + 1, SC_US));
    sc_assert(read32(OFS_SYST_CVR) == 99);
    sc_assert(read32(OFS_SYST_CSR) ==
              (SYST_CSR_COUNTFLAG | SYST_CSR_ENABLE | SYST_CSR_CLKSOURCE));

    // COUNTFLAG should clear on read
    sc_assert(read32(OFS_SYST_CSR) == (SYST_CSR_ENABLE | SYST_CSR_CLKSOURCE));

    // Test -- Enable interrupt
    write32(OFS_SYST_CSR, SYST_CSR_TICKINT | SYST_CSR_ENABLE);
    wait(sc_time(10, SC_US));
    sc_assert(test.irq.read() == false);
    wait(sc_time(90, SC_US));
    sc_assert(test.irq.read() == true);

    // Test -- Retire interrupt
    test.returning_exception.write(SYST_EXCEPT_ID);
    wait(sc_time(1, SC_US));
    sc_assert(test.irq.read() == false);

    sc_stop();
  }

  void write32(const uint32_t addr, const uint32_t val, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[4];
    trans.set_data_ptr(data);
    trans.set_data_length(4);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);

    Utility::unpackBytes(data, Utility::htotl(val), 4);
    test.iSocket->b_transport(trans, delay);
    if (doWait) {
      wait(delay);
    }
  }

  uint32_t read32(const uint32_t addr, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[4];
    trans.set_data_ptr(data);
    trans.set_data_length(4);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);
    test.iSocket->b_transport(trans, delay);

    if (doWait) {
      wait(delay);
    }
    return Utility::ttohl(Utility::packBytes(data, 4));
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
