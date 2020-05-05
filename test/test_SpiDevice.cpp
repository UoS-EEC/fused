/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <systemc>
#include <tlm>
#include "mcu/SpiTransactionExtension.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "ps/EventLog.hpp"
#include "sd/DummySpiDevice.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
 public:
  tlm_utils::simple_initiator_socket<dut> iSpiSocket{"iSocket"};

  SC_CTOR(dut) { m_dut.tSpiSocket.bind(iSpiSocket); }

  DummySpiDevice m_dut{"dut"};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    // Prepare payload object
    tlm::tlm_generic_payload trans;
    SpiTransactionExtension spiExtension;

    uint8_t data = 0xB0;

    trans.set_extension(&spiExtension);
    trans.set_address(0);      // SPI doesn't use address
    trans.set_data_length(1);  // Transfer size up to 1 byte
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_data_ptr(&data);
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    sc_time delay = sc_core::sc_time(80, sc_core::SC_US);

    wait(SC_ZERO_TIME);

    std::cout << "TESTING STARTS" << std::endl;
    // ------ TEST: SPI Operation
    // Blocking transport call
    test.iSpiSocket->b_transport(trans, delay);

    wait(delay);

    // Check response status
    if (trans.is_response_error()) {
      SC_REPORT_FATAL(this->name(), "Response error");
    }

    std::cout << std::endl << "TESTING DONE" << std::endl;
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
