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
#include "ps/PowerModelChannel.hpp"
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
  sc_signal_resolved chipSelect{"chipSelect"};  //! Active low
  PowerModelChannel powerModelChannel{"powerModelChannel", "/tmp",
                                      sc_time(1, SC_US)};

  // Sockets
  tlm_utils::simple_initiator_socket<dut> iSpiSocket{"iSpiSocket"};

  SC_CTOR(dut) {
    m_dut.nReset.bind(nReset);
    m_dut.chipSelect.bind(chipSelect);
    m_dut.tSocket.bind(iSpiSocket);
    m_dut.powerModelPort.bind(powerModelChannel);
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
    // One measurement of each sensor, in forced mode
    spiWrite(Bme280::ADDR_CTRL_HUM, 1);
    spiWrite(Bme280::ADDR_CTRL_MEAS, (1u << 5) | (1u << 2) | 2u);
    wait(1.01, SC_MS);
    sc_assert(spiRead(Bme280::ADDR_STATUS, 1)[0] == (1u << 3));
    wait(2 + 2.5 + 2.5 + 0.5, SC_MS);
    sc_assert(spiRead(Bme280::ADDR_STATUS, 1)[0] == 0);

    spdlog::info("TEST: Start continuous (normal-mode) measurement");
    // 10 ms standby time, iir coefficient = 4
    spiWrite(Bme280::ADDR_CONFIG, (0b110u << 5) | (0b010 << 2));
    // Normal-mode, 2 of each measurement
    spiWrite(Bme280::ADDR_CTRL_HUM, 2);
    spiWrite(Bme280::ADDR_CTRL_MEAS, (2u << 5) | (2u << 2) | 3u);
    wait(1.01, SC_MS);
    sc_assert(spiRead(Bme280::ADDR_STATUS, 1)[0] == (1u << 3));
    wait(100.0, SC_MS);
    // Set to sleep mode
    spiWrite(Bme280::ADDR_CTRL_MEAS, (2u << 5) | (2u << 2) | 0u);
    wait(250.0, SC_MS);
    sc_assert(spiRead(Bme280::ADDR_STATUS, 1)[0] == 0);

    spdlog::info("TEST: Burst-read");
    auto measurements = spiRead(Bme280::ADDR_PRESS_MSB, 8);

    unsigned pressure = (static_cast<unsigned>(measurements[0]) << 12) |
                        (static_cast<unsigned>(measurements[1]) << 4) |
                        static_cast<unsigned>(measurements[2]);
    unsigned temperature = (static_cast<unsigned>(measurements[3]) << 12) |
                           (static_cast<unsigned>(measurements[4]) << 4) |
                           static_cast<unsigned>(measurements[5]);
    unsigned humidity = (static_cast<unsigned>(measurements[6]) << 8) |
                        static_cast<unsigned>(measurements[7]);
    spdlog::info("Pressure = 0x{:08x} ({:.3f} hPa)", pressure,
                 (pressure * Bme280::PRESS_SCALE) + Bme280::PRESS_OFFSET);
    spdlog::info("Temperature = 0x{:08x} ({:.3f} C)", temperature,
                 (temperature * Bme280::TEMP_SCALE) + Bme280::TEMP_OFFSET);
    spdlog::info("Humidity = 0x{:08x} ({:.3f} %RH)", humidity,
                 (humidity * Bme280::HUM_SCALE) + Bme280::HUM_OFFSET);

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
    test.chipSelect.write(sc_dt::SC_LOGIC_0);
    wait(SC_ZERO_TIME);
    trans.set_data_ptr(&addr);
    test.iSpiSocket->b_transport(trans, delay);  // Transfer address
    wait(delay);
    trans.set_data_ptr(&cmd);
    test.iSpiSocket->b_transport(trans, delay);  // Transfer command
    wait(delay);
    test.chipSelect.write(sc_dt::SC_LOGIC_1);
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
    test.chipSelect.write(sc_dt::SC_LOGIC_0);
    wait(SC_ZERO_TIME);
    test.iSpiSocket->b_transport(trans, delay);  // Transfer address
    wait(delay);
    for (int i = 0; i < len; ++i) {
      test.iSpiSocket->b_transport(trans, delay);  // Transfer data
      response[i] = spiExtension->response;
      wait(delay);
    }
    test.chipSelect.write(sc_dt::SC_LOGIC_1);
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

  tester t("tester");
  sc_start();
  return 0;
}
