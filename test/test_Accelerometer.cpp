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
#include "sd/Accelerometer.hpp"
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
  sc_signal_resolved irq{"irq"};                //! Accelerometer irq output

  // Sockets
  tlm_utils::simple_initiator_socket<dut> iSpiSocket{"iSpiSocket"};

  SC_CTOR(dut) {
    m_dut.nReset.bind(nReset);
    m_dut.chipSelect.bind(chipSelect);
    m_dut.tSocket.bind(iSpiSocket);
    m_dut.irq.bind(irq);
  }

  Accelerometer m_dut{"dut"};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    spdlog::info("Testing Accelerometer");
    resetDut();
    wait(SC_ZERO_TIME);

    //-------------------------------------------------------------------------
    spdlog::info("TEST: Read & check reset values");
    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL, 1)[0] == 0x00);
    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL_FS, 1)[0] == 0x00);
    sc_assert(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] ==
              Accelerometer::BitMasks::STATUS_BUSY);
    sc_assert(spiRead(Accelerometer::RegisterAddress::DATA, 1)[0] == 0x00);
    sc_assert(spiRead(Accelerometer::RegisterAddress::FIFO_THR, 1)[0] == 0x00);

    //-------------------------------------------------------------------------
    spdlog::info("TEST: Sleep -> Standby");
    resetDut();
    sc_assert(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] ==
              Accelerometer::BitMasks::STATUS_BUSY);
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_STANDBY);
    wait(sc_time::from_seconds(Accelerometer::DELAY_SLEEP_TO_STANDBY));
    sc_assert(!(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
                Accelerometer::BitMasks::STATUS_BUSY));

    //-------------------------------------------------------------------------
    spdlog::info("TEST: Single-shot measurement");
    resetDut();

    // Move to standby mode
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_STANDBY);
    wait(sc_time::from_seconds(Accelerometer::DELAY_SLEEP_TO_STANDBY));
    sc_assert(!(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
                Accelerometer::BitMasks::STATUS_BUSY));

    // 10 ms sampling time
    spiWrite(Accelerometer::RegisterAddress::CTRL_FS, 100);

    // Start sampling all axes
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_SINGLE |
                 Accelerometer::BitMasks::CTRL_X_EN |
                 Accelerometer::BitMasks::CTRL_Y_EN |
                 Accelerometer::BitMasks::CTRL_Z_EN);

    // Check busy==true while measurement ongoing
    wait(sc_time(100, SC_US));
    sc_assert(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
              Accelerometer::BitMasks::STATUS_BUSY);

    // Check busy==false && Mode==Standby after measurement
    wait(sc_time(10.1, SC_MS));
    sc_assert(!(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
                Accelerometer::BitMasks::STATUS_BUSY));

    // Check Mode==Standby
    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL, 1)[0] &
              Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    // Load data
    // header + x axis + y axis + z axis
    auto resvec = spiRead(Accelerometer::RegisterAddress::DATA, 1 + 1 + 1 + 1);
    /*
    spdlog::info("Resvec= :");
    for (const auto &it : resvec) {
      std::cout << (int)it << '\n';
    }
    */
    sc_assert(resvec[0] == 7);   // header
    sc_assert(resvec[1] == 0);   // x
    sc_assert(resvec[2] == 0);   // y
    sc_assert(resvec[3] == 62);  // z

    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL, 1)[0] &
              Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    //-------------------------------------------------------------------------
    spdlog::info("TEST: Single-shot measurement with interrupt");
    resetDut();

    // Move to standby mode
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_STANDBY);
    wait(sc_time::from_seconds(Accelerometer::DELAY_SLEEP_TO_STANDBY));
    sc_assert(!(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
                Accelerometer::BitMasks::STATUS_BUSY));

    // 10 ms sampling time
    spiWrite(Accelerometer::RegisterAddress::CTRL_FS, 100);

    // Start sampling all axes, with interrupt enabled
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_SINGLE |
                 Accelerometer::BitMasks::CTRL_IE |
                 Accelerometer::BitMasks::CTRL_X_EN |
                 Accelerometer::BitMasks::CTRL_Y_EN |
                 Accelerometer::BitMasks::CTRL_Z_EN);

    // Check busy==true while measurement ongoing
    wait(sc_time(100, SC_US));
    sc_assert(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
              Accelerometer::BitMasks::STATUS_BUSY);

    // Wait for irq pulse
    sc_event timeout{"timeout"};
    timeout.notify(sc_time(11, SC_MS));

    wait(timeout | test.irq.posedge_event());
    sc_assert(test.irq.read() == sc_dt::SC_LOGIC_1);

    // Check busy==false && Mode==Standby after measurement
    wait(sc_time(0.1, SC_MS));
    sc_assert(!(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
                Accelerometer::BitMasks::STATUS_BUSY));

    // Check Mode==Standby
    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL, 1)[0] &
              Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    // Load data

    // header + x axis + y axis + z axis
    resvec = spiRead(Accelerometer::RegisterAddress::DATA, 1 + 1 + 1 + 1);

    // Irq should be cleared after loading data
    wait(sc_time(1, SC_US));
    sc_assert(test.irq.read() == sc_dt::SC_LOGIC_0);

    /*
    spdlog::info("Resvec= :");
    for (const auto &it : resvec) {
      std::cout << (int)it << '\n';
    }
    */
    sc_assert(resvec[0] == 7);   // header
    sc_assert(resvec[1] == 0);   // x
    sc_assert(resvec[2] == 0);   // y
    sc_assert(resvec[3] == 62);  // z

    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL, 1)[0] &
              Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    //-------------------------------------------------------------------------
    spdlog::info("TEST: Continuous measurement");
    resetDut();

    // Move to standby mode
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_STANDBY);
    wait(sc_time::from_seconds(Accelerometer::DELAY_SLEEP_TO_STANDBY));
    sc_assert(!(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
                Accelerometer::BitMasks::STATUS_BUSY));

    // 10 ms sampling time
    spiWrite(Accelerometer::RegisterAddress::CTRL_FS, 100);

    // Set threshold to 128 bytes
    spiWrite(Accelerometer::RegisterAddress::FIFO_THR, 128 / 4);

    // Start sampling all axes in continuous mode, with interrupt enabled
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_CONTINUOUS |
                 Accelerometer::BitMasks::CTRL_IE |
                 Accelerometer::BitMasks::CTRL_X_EN |
                 Accelerometer::BitMasks::CTRL_Y_EN |
                 Accelerometer::BitMasks::CTRL_Z_EN);

    // Check busy==true while measurement ongoing
    wait(sc_time(100, SC_US));
    sc_assert(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
              Accelerometer::BitMasks::STATUS_BUSY);

    // Wait for irq pulse
    timeout.cancel();
    timeout.notify(sc_time(1 + 10 * ((128 >> 2) + 1), SC_MS));

    wait(timeout | test.irq.posedge_event());
    sc_assert(test.irq.read() == sc_dt::SC_LOGIC_1);

    // Stop measurement
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    // Load data
    for (int i = 0; i < 128 / 4; i++) {
      resvec = spiRead(Accelerometer::RegisterAddress::DATA, 1 + 1 + 1 + 1);
      /*
      spdlog::info("Resvec= :");
      for (const auto &it : resvec) {
        std::cout << (int)it << '\n';
      }
      */
      sc_assert(resvec[0] == 7);   // header
      sc_assert(resvec[1] == 0);   // x
      sc_assert(resvec[2] == 0);   // y
      sc_assert(resvec[3] == 62);  // z
    }

    // Irq shuold be cleared by now
    sc_assert(test.irq.read() == sc_dt::sc_logic_0);

    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL, 1)[0] &
              Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    //-------------------------------------------------------------------------
    spdlog::info("TEST: Continuous measurement with overflow");
    resetDut();

    // Move to standby mode
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_STANDBY);
    wait(sc_time::from_seconds(Accelerometer::DELAY_SLEEP_TO_STANDBY));
    sc_assert(!(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
                Accelerometer::BitMasks::STATUS_BUSY));

    // 10 ms sampling time
    spiWrite(Accelerometer::RegisterAddress::CTRL_FS, 100);

    // Set threshold to 1000 bytes
    spiWrite(Accelerometer::RegisterAddress::FIFO_THR, 1000 / 4);

    // Start sampling all axes in continuous mode, with interrupt enabled
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_CONTINUOUS |
                 Accelerometer::BitMasks::CTRL_IE |
                 Accelerometer::BitMasks::CTRL_X_EN |
                 Accelerometer::BitMasks::CTRL_Y_EN |
                 Accelerometer::BitMasks::CTRL_Z_EN);

    // Check busy==true while measurement ongoing
    wait(sc_time(100, SC_US));
    sc_assert(spiRead(Accelerometer::RegisterAddress::STATUS, 1)[0] &
              Accelerometer::BitMasks::STATUS_BUSY);

    // Wait for irq pulse
    timeout.cancel();
    timeout.notify(sc_time(1 + 10 * 256, SC_MS));
    wait(timeout | test.irq.posedge_event());
    sc_assert(test.irq.read() == sc_dt::SC_LOGIC_1);

    // Wait a bit longer to let FIFO overflow
    wait(sc_time(100, SC_MS));

    // Stop measurement
    spiWrite(Accelerometer::RegisterAddress::CTRL,
             Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    // Load data
    for (int i = 0; i < 256; i++) {
      resvec = spiRead(Accelerometer::RegisterAddress::DATA, 1 + 1 + 1 + 1);
      /*
      spdlog::info("Resvec= :");
      for (const auto &it : resvec) {
        std::cout << (int)it << '\n';
      }
      */
      sc_assert(resvec[0] == 7);   // header
      sc_assert(resvec[1] == 0);   // x
      sc_assert(resvec[2] == 0);   // y
      sc_assert(resvec[3] == 62);  // z
    }

    // Irq shuold be cleared by now
    sc_assert(test.irq.read() == sc_dt::sc_logic_0);

    sc_assert(spiRead(Accelerometer::RegisterAddress::CTRL, 1)[0] &
              Accelerometer::BitMasks::CTRL_MODE_STANDBY);

    sc_stop();
  }

  void resetDut() {
    test.nReset.write(false);
    wait(sc_time(1, SC_US));
    test.nReset.write(true);
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
      // spdlog::info("spiRead::Response: 0x{:02x}", response[i]);
      wait(delay);
    }
    test.chipSelect.write(sc_dt::SC_LOGIC_1);
    wait(SC_ZERO_TIME);

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
