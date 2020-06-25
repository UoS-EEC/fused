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
#include <vector>
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

    test.nReset.write(true);
    wait(SC_ZERO_TIME);

    spdlog::info("TEST: Read & check reset values of registers");
    sc_assert(spiRead(Bme280::ADDR_ID, 1)[0] == 0x60);
    sc_assert(spiRead(Bme280::ADDR_RESET, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_CTRL_HUM, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_STATUS, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_CTRL_MEAS, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_CONFIG, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_PRESS_MSB, 1)[0] == 0x80);
    sc_assert(spiRead(Bme280::ADDR_PRESS_LSB, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_PRESS_XLSB, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_TEMP_MSB, 1)[0] == 0x80);
    sc_assert(spiRead(Bme280::ADDR_TEMP_LSB, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_TEMP_XLSB, 1)[0] == 0x00);
    sc_assert(spiRead(Bme280::ADDR_HUM_MSB, 1)[0] == 0x80);
    sc_assert(spiRead(Bme280::ADDR_HUM_LSB, 1)[0] == 0x00);

    spdlog::info("TEST: Start basic measurement");

    // 1 measurement of each sensor, in forced mode
    spiWrite(Bme280::ADDR_CTRL_HUM, 1);
    spiWrite(Bme280::ADDR_CTRL_MEAS, (1u << 5) | (1u << 2) | 2u);
    wait(1.01, SC_MS);
    sc_assert(spiRead(Bme280::ADDR_STATUS, 1)[0] == (1u << 3));
    wait(2 + 2.5 + 2.5 + 0.5, SC_MS);
    sc_assert(spiRead(Bme280::ADDR_STATUS, 1)[0] == 0);

    sc_stop();
  }

  void spiWrite(uint8_t addr, uint8_t cmd) {
    // Prepare payload object
    tlm::tlm_generic_payload trans;

    // Clear read bit to signal write command
    addr &= ~(1u << 7);

    auto *spiExtension = new SpiTransactionExtension();

    spiExtension->clkPeriod = sc_core::sc_time(10, sc_core::SC_US);
    spiExtension->nDataBits = 8;
    spiExtension->phase = SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE;
    spiExtension->polarity = SpiTransactionExtension::SpiPolarity::HIGH;
    spiExtension->bitOrder = SpiTransactionExtension::SpiBitOrder::MSB_FIRST;

    trans.set_extension(spiExtension);
    trans.set_address(0);      // SPI doesn't use address
    trans.set_data_length(1);  // Transfer size is 1 byte
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    sc_time delay = spiExtension->transferTime();

    // Start transfer
    test.chipSelect.write(false);
    wait(SC_ZERO_TIME);
    trans.set_data_ptr(&addr);
    test.iSpiSocket->b_transport(trans, delay);  // Transfer address
    wait(delay);
    trans.set_data_ptr(&cmd);
    test.iSpiSocket->b_transport(trans, delay);  // Transfer command
    wait(delay);
    test.chipSelect.write(true);
    wait(SC_ZERO_TIME);

    // delete spiExtension;
  }

  std::vector<uint8_t> spiRead(const uint8_t addr, const size_t len) {
    // Prepare payload object
    uint8_t data = addr | READ_BIT;
    tlm::tlm_generic_payload trans;

    auto *spiExtension = new SpiTransactionExtension();

    spiExtension->clkPeriod = sc_core::sc_time(10, sc_core::SC_US);
    spiExtension->nDataBits = 8;
    spiExtension->phase = SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE;
    spiExtension->polarity = SpiTransactionExtension::SpiPolarity::HIGH;
    spiExtension->bitOrder = SpiTransactionExtension::SpiBitOrder::MSB_FIRST;

    trans.set_extension(spiExtension);
    trans.set_address(0);      // SPI doesn't use address
    trans.set_data_length(1);  // Transfer size is 1 byte
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_data_ptr(&data);
    sc_time delay = spiExtension->transferTime();

    // Start transfer
    std::vector<uint8_t> response(len);
    test.chipSelect.write(false);
    wait(SC_ZERO_TIME);
    test.iSpiSocket->b_transport(trans, delay);  // Transfer address
    wait(delay);
    for (int i = 0; i < len; ++i) {
      test.iSpiSocket->b_transport(trans, delay);  // Transfer data
      response[i] = spiExtension->response;
      spdlog::info("spiRead: received data 0x{:02x}", response[i]);
      wait(delay);
    }
    test.chipSelect.write(true);
    wait(SC_ZERO_TIME);

    // delete spiExtension;
    return response;
  }

  static const unsigned READ_BIT = (1u << 7);

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
