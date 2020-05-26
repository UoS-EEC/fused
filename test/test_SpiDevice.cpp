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

#include "mcu/RegisterFile.hpp"
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
  // Signals
  sc_signal<bool> pwrGood{"pwrGood"};
  sc_signal<bool> chipSelect{"chipSelect"};  //! Active low

  // Sockets
  tlm_utils::simple_initiator_socket<dut> iSpiSocket{"iSpiSocket"};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.chipSelect.bind(chipSelect);
    m_dut.tSocket.bind(iSpiSocket);
  }

  DummySpiDevice m_dut{"dut"};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    // Prepare payload object
    tlm::tlm_generic_payload trans;
    SpiTransactionExtension spiExtension;

    uint8_t data = 0x0b;

    trans.set_extension(&spiExtension);
    trans.set_address(0);      // SPI doesn't use address
    trans.set_data_length(1);  // Transfer size up to 1 byte

    spiExtension.clkPeriod = sc_core::sc_time(10, sc_core::SC_US);
    spiExtension.nDataBits = 8;
    spiExtension.phase = SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE;
    spiExtension.polarity = SpiTransactionExtension::SpiPolarity::HIGH;
    spiExtension.bitOrder = SpiTransactionExtension::SpiBitOrder::MSB_FIRST;
    sc_time delay = spiExtension.transferTime();

    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_data_ptr(&data);
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    test.pwrGood.write(true);
    test.chipSelect.write(false);
    wait(SC_ZERO_TIME);

    std::cout << "TESTING STARTS" << std::endl;

    // ------ TEST: SPI Operation
    // Blocking transport call
    test.iSpiSocket->b_transport(trans, delay);

    wait(delay);

    // Check response status
    if (trans.is_response_error()) {
      SC_REPORT_FATAL(this->name(), "Response error");
    } else {
      sc_assert(trans.get_response_status() ==
                tlm::tlm_response_status::TLM_OK_RESPONSE);
    }

    // Check for correct payload
    // First returned payload should be reset (all zeros)
    sc_assert(spiExtension.response == 0x00);

    // Second SPI transaction
    data = 0x0C;
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    test.iSpiSocket->b_transport(trans, delay);

    wait(delay);

    // Check response status
    if (trans.is_response_error()) {
      SC_REPORT_FATAL(this->name(), "Response error");
    } else {
      sc_assert(trans.get_response_status() ==
                tlm::tlm_response_status::TLM_OK_RESPONSE);
    }

    // Second returned payload should be first payload (0x0B)
    sc_assert(spiExtension.response == 0x0b);

    // Pull N chip select high
    test.chipSelect.write(true);
    wait(SC_ZERO_TIME);
    // Try sending something
    data = 0x0d;
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
    test.iSpiSocket->b_transport(trans, delay);

    wait(delay);

    // Check response status
    sc_assert(trans.get_response_status() ==
              tlm::tlm_response_status::TLM_INCOMPLETE_RESPONSE);

    // Pull N chip select low
    test.chipSelect.write(false);
    wait(SC_ZERO_TIME);
    test.iSpiSocket->b_transport(trans, delay);

    wait(delay);

    // Check response status
    sc_assert(trans.get_response_status() ==
              tlm::tlm_response_status::TLM_OK_RESPONSE);
    sc_assert(spiExtension.response == 0x0c);

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
