/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/msp430fr5xx/ClockSystem.hpp"
#include "ps/PowerModelChannel.hpp"
#include "utilities/Config.hpp"
#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

SC_MODULE(dut) {
public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood"};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel mclk_sig{"mclk_sig"};
  ClockSourceChannel smclk_sig{"smclk_sig"};
  ClockSourceChannel aclk_sig{"aclk_sig"};
  ClockSourceChannel vloclk_sig{"vloclk_sig"};
  ClockSourceChannel modclk_sig{"modclk_sig"};
  ClockSourceChannel sysClk{"sysClk", sc_time(1, SC_NS)};
  PowerModelChannel powerModelChannel{"powerModelChannel", "/tmp",
                                      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.tSocket.bind(iSocket);
    m_dut.mclk.bind(mclk_sig);
    m_dut.smclk.bind(smclk_sig);
    m_dut.aclk.bind(aclk_sig);
    m_dut.vloclk.bind(vloclk_sig);
    m_dut.modclk.bind(modclk_sig);
    m_dut.systemClk.bind(sysClk);
    m_dut.powerModelPort.bind(powerModelChannel);
  }

  ClockSystem m_dut{"dut", 0};
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

    // TEST -  Default/Reset condition
    wait(SC_ZERO_TIME);
    sc_assert(test.vloclk_sig.getPeriod() == sc_time(100.0, SC_US));
    sc_assert(test.modclk_sig.getPeriod() == sc_time(200.0, SC_NS));
    sc_assert(test.mclk_sig.getPeriod() == sc_time(1, SC_US));
    sc_assert(test.smclk_sig.getPeriod() == sc_time(1, SC_US));
    sc_assert(test.aclk_sig.getPeriod() == sc_time(100, SC_US));

    // TEST - Unlock registers
    data[0] = 0x00;
    data[1] = 0xa5;
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(OFS_CSCTL0);
    test.iSocket->b_transport(trans, delay);
    sc_assert(trans.get_response_status() == tlm::TLM_OK_RESPONSE);

    // TEST -  Clock frequencies

    sc_stop();
  }

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  // Set up paths
  // Parse CLI arguments & config file
  Config::get().parseFile("../config/Msp430TestBoard-config.yml");

  tester t("tester");
  sc_start();
  return false;
}
