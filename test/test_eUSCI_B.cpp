/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <array>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"
#include "mcu/msp430fr5xx/eUSCI_B.hpp"

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
 public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood"};
  sc_signal<bool> irq{"irq"};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  tlm_utils::simple_target_socket<dut> tSocket{"tSocket"};
  ClockSourceChannel smclk{"smclk", sc_time(1, SC_US)};
  ClockSourceChannel aclk{"aclk", sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.tSocket.bind(iSocket);
    m_dut.iSocket.bind(tSocket);
    m_dut.aclk.bind(aclk);
    m_dut.smclk.bind(smclk);
    m_dut.irq.bind(irq);
    irq.write(false);
  }

  eUSCI_B m_dut{"dut",0 , 0x2f,  sc_time(1, SC_NS)};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // ------ TEST: ISER Write-one-to-set
    test.m_dut.reset();
/*
    write32(OFS_NVIC_ISER, 0xAAAAAAAA, false);
    write32(OFS_NVIC_ISER, 0x00000000, false);  // Should have no effect
    sc_assert(read32(OFS_NVIC_ISER) == 0xAAAAAAAA);
    sc_assert(read32(OFS_NVIC_ICER) == 0xAAAAAAAA);
*/

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
