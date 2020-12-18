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
#include "mcu/msp430fr5xx/DigitalIo.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "ps/PowerModelEventChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

SC_MODULE(dut) {
 public:
  std::array<sc_signal_resolved, 16> port;  // Digital IO port
  sc_signal<bool> irq0{"irq0"};
  sc_signal<bool> irq1{"irq1"};
  sc_signal<bool> pwrGood{"pwrGood"};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel clk{"clk", sc_time(1, SC_NS)};
  PowerModelEventChannel powerModelEventChannel{
      "powerModelEventChannel", "/tmp/testPowerModelChannel.csv",
      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    for (unsigned int i = 0; i < port.size(); i++) {
      m_dut.pins[i].bind(port[i]);
    }
    m_dut.pwrOn.bind(pwrGood);
    m_dut.irq[0].bind(irq0);
    m_dut.irq[1].bind(irq1);
    m_dut.tSocket.bind(iSocket);
    m_dut.systemClk.bind(clk);
    m_dut.powerModelEventPort.bind(powerModelEventChannel);
  }

  DigitalIo m_dut{"port", 0, 0x1f};
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
    trans.set_command(tlm::TLM_WRITE_COMMAND);

    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // ------ TEST: Set all pins to 1
    test.m_dut.reset();
    delay = SC_ZERO_TIME;
    writeWord(OFS_PADIR, 0xffff);
    writeWord(OFS_PAOUT, 0xffff);

    wait(sc_time(1, SC_NS));  // Wait for pins to be updated
    for (unsigned i = 0; i < test.port.size(); i++) {
      sc_assert(test.port[i].read().to_bool() == true);
    }

    // ------ TEST: Set all pins to 0
    test.m_dut.reset();
    writeWord(OFS_PADIR, 0xffff);
    writeWord(OFS_PAOUT, 0);

    wait(sc_time(1, SC_NS));  // Wait for pins to be updated
    for (unsigned i = 0; i < test.port.size(); i++) {
      sc_assert(test.port[i].read().to_bool() == false);
    }

    // ------ TEST: Test direction register
    test.m_dut.reset();
    writeWord(OFS_PADIR, 0x5555);
    writeWord(OFS_PAOUT, 0xffff);

    wait(sc_time(1, SC_NS));  // Wait for pins to be updated
    bool shouldBeSet = true;
    for (unsigned int i = 0; i < test.port.size(); i++) {
      sc_assert(test.port[i].read().to_bool() == shouldBeSet);
      shouldBeSet = !shouldBeSet;
    }

    // ------ TEST: Test correct byte order
    test.m_dut.reset();
    writeWord(OFS_PADIR, 0xffff);
    writeWord(OFS_PAOUT, 0x00ff);

    for (unsigned int i = 0; i < test.port.size(); i++) {
      if (i < 8) {
        sc_assert(test.port[i].read().to_bool() == true);
      } else {
        sc_assert(test.port[i].read().to_bool() == false);
      }
    }

    // ------ TEST: test byte access correct byte order
    test.m_dut.reset();
    delay = SC_ZERO_TIME;
    trans.set_data_length(1);
    trans.set_command(tlm::TLM_WRITE_COMMAND);

    // Direction register
    data[0] = 0xff;
    trans.set_command(tlm::TLM_WRITE_COMMAND);

    trans.set_address(OFS_PADIR);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

    trans.set_address(OFS_PADIR + 1);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

    // Output register
    data[0] = 0xff;
    trans.set_address(OFS_PAOUT);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

    data[0] = 0x00;
    trans.set_address(OFS_PAOUT + 1);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

    wait(delay);
    for (unsigned int i = 0; i < test.port.size(); i++) {
      if (i < 8) {
        sc_assert(test.port[i].read().to_bool() == true);
      } else {
        sc_assert(test.port[i].read().to_bool() == false);
      }
    }

    spdlog::info("Test successful.");
    sc_stop();
  }

  void writeWord(const uint32_t addr, const uint32_t val,
                 const bool doWait = true) {
    sc_assert(TARGET_WORD_SIZE == 2);
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[2];
    trans.set_data_ptr(data);
    trans.set_data_length(2);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);
    Utility::unpackBytes(data, Utility::htots(val), 2);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

    if (doWait) {
      wait(delay);
    }
  }

  uint32_t readWord(const uint32_t addr, const bool doWait = true) {
    sc_assert(TARGET_WORD_SIZE == 2);
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[2];
    trans.set_data_ptr(data);
    trans.set_data_length(2);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

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
