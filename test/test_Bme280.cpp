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
#include "sd/Bme280.hpp"
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
  sc_signal<bool> nReset{"nReset"};
  sc_signal<bool> chipSelect{"chipSelect"};  //! Active low

  // Sockets
  tlm_utils::simple_initiator_socket<dut> iSpiSocket{"iSpiSocket"};

  SC_CTOR(dut) {
    m_dut.nReset.bind(nReset);
    m_dut.chipSelect.bind(chipSelect);
    m_dut.tSocket.bind(iSpiSocket);
  }

  Bme280 m_dut{"dut"};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    spdlog::info("Testing Bme280");
    // Prepare payload object
    uint8_t data = 0x00;
    tlm::tlm_generic_payload trans;
    auto *spiExtension = new SpiTransactionExtension();

    trans.set_extension(spiExtension);
    trans.set_address(0);      // SPI doesn't use address
    trans.set_data_length(1);  // Transfer size is 1 byte
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_data_ptr(&data);

    spiExtension->clkPeriod = sc_core::sc_time(10, sc_core::SC_US);
    spiExtension->nDataBits = 8;
    spiExtension->phase = SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE;
    spiExtension->polarity = SpiTransactionExtension::SpiPolarity::HIGH;
    spiExtension->bitOrder = SpiTransactionExtension::SpiBitOrder::MSB_FIRST;
    sc_time delay = spiExtension->transferTime();

    test.nReset.write(true);
    test.chipSelect.write(false);
    wait(SC_ZERO_TIME);

    const unsigned READ_BIT = (1u << 7);

    spdlog::info("TEST: Read ID register");
    data = Bme280::ADDR_ID | READ_BIT;           // ID register address
    test.iSpiSocket->b_transport(trans, delay);  // Transfer address
    wait(delay);
    test.iSpiSocket->b_transport(trans, delay);  // Get ID
    wait(delay);
    spdlog::info("Response: 0x{:08x}", spiExtension->response);
    sc_assert(spiExtension->response == 0x60);

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
  return 0;
}
