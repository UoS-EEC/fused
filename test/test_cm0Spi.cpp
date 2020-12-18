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
#include "include/cm0-fused.h"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/SpiTransactionExtension.hpp"
#include "mcu/cortex-m0/Spi.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "ps/PowerModelEventChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
 public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood"};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  tlm_utils::simple_target_socket<dut> spiSocket{"spiSocket"};
  sc_signal<int> active_exception{"active_exception", -1};
  sc_signal<bool> irq{"irq"};
  ClockSourceChannel spiclk{"spiclk", sc_time(1, SC_US)};
  ClockSourceChannel sysclk{"sysclk", sc_time(1, SC_NS)};
  PowerModelEventChannel powerModelEventChannel{
      "powerModelEventChannel", "/tmp/testPowerModelChannel.csv",
      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.systemClk.bind(sysclk);
    m_dut.tSocket.bind(iSocket);
    m_dut.spiSocket.bind(spiSocket);
    m_dut.clk.bind(spiclk);
    m_dut.irq.bind(irq);
    m_dut.active_exception.bind(active_exception);
    spiSocket.register_b_transport(this, &dut::b_transport);
    m_dut.powerModelEventPort.bind(powerModelEventChannel);
  }

  virtual void b_transport(tlm::tlm_generic_payload & trans, sc_time & delay) {
    auto *ptr = trans.get_data_ptr();
    auto len = trans.get_data_length();

    // SPI peripheral device checks if params and clock valid
    // then respond
    auto *spiExtension = trans.get_extension<SpiTransactionExtension>();
    spiExtension->response = 0x00CD;
    m_lastTransaction.deep_copy_from(trans);  // Copy the transaction object
    m_lastPayload = trans.get_data_ptr()[0];  // Copy payload data
    spdlog::info("SPI transaction with data 0x{:02x}", m_lastPayload);
    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  }

  bool checkPayload(const unsigned c) const { return m_lastPayload == c; }

  Spi m_dut{"dut", 0, 0xff};

  // Variables
  tlm::tlm_generic_payload m_lastTransaction;
  uint8_t m_lastPayload;
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // ------ TEST: Initialization and Reset
    // Power on reset
    spdlog::info("Testing reset state...");
    sc_assert(read16(OFS_SPI_CR1) == 0);
    sc_assert(read16(OFS_SPI_CR2) == 0x0700);
    sc_assert(read16(OFS_SPI_SR) == 0x0002);
    sc_assert(read16(OFS_SPI_CRCPR) == 0x0007);
    sc_assert(read16(OFS_SPI_RXCRCR) == 0);
    sc_assert(read16(OFS_SPI_TXCRCR) == 0);
    spdlog::info("SUCCESS");

    // ------ TEST: SPI basic transfer
    spdlog::info("Testing basic SPI transfer...");
    // Configure SPI parameters
    // set phase, active high, 8-bit, master, 3-pin, aclk
    write16(OFS_SPI_CR2, (7u << Spi::DS_SHIFT));  // 8-bit data size
    write16(OFS_SPI_CR1,
            (1u << Spi::BR_SHIFT) |        // Baudrate = clk/4
                (1u << Spi::SPE_SHIFT) |   // Enable
                (1u << Spi::MSTR_SHIFT));  // Master mode
    write16(OFS_SPI_DR, 0xabcd);           // Write data -> trigger transaction
    wait(sc_time(1, SC_US));
    auto sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(!(sr & Spi::RXNE_MASK));
    wait(sc_time(30, SC_US));
    sc_assert(test.checkPayload(0x00cd));
    wait(sc_time(34, SC_US));
    sc_assert(test.checkPayload(0x00ab));
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(!(read16(OFS_SPI_SR) & Spi::BSY_MASK));
    // Empty rx register
    auto val = read16(OFS_SPI_DR);
    sc_assert(val == 0xcdcd);
    sr = read16(OFS_SPI_SR);
    sc_assert(!(sr & Spi::RXNE_MASK));

    // Check SPI packet parameters
    auto ext = test.m_lastTransaction.get_extension<SpiTransactionExtension>();
    sc_assert(ext->clkPeriod == sc_time(4, SC_US));
    sc_assert(ext->phase ==
              SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE);
    sc_assert(ext->polarity == SpiTransactionExtension::SpiPolarity::LOW);
    wait(sc_time(80, SC_US));
    spdlog::info("SUCCESS");

    // ------ TEST: TXE interrupt
    spdlog::info("Testing TXE interrupt...");
    test.m_dut.reset();
    write16(OFS_SPI_CR2,
            Spi::TXEIE_MASK |            // TXE interrupt enable
                (7u << Spi::DS_SHIFT));  // 8-bit data size
    write16(OFS_SPI_CR1,
            (1u << Spi::BR_SHIFT) |        // Baudrate = clk/4
                (1u << Spi::SPE_SHIFT) |   // Enable
                (1u << Spi::MSTR_SHIFT));  // Master mode
    write16(OFS_SPI_DR, 0xabcd);           // Write data -> trigger transaction
    write16(OFS_SPI_DR, 0xef01);           // Fill tx buffer

    wait(sc_time(1, SC_US));  // Start send byte (1/4)
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(!(sr & Spi::TXE_MASK));
    sc_assert(!(sr & Spi::RXNE_MASK));
    sc_assert(test.checkPayload(0x00cd));
    sc_assert(test.irq.read() == 0);

    wait(sc_time(32, SC_US));  // Start send byte (2/4)
    sc_assert(test.checkPayload(0x00ab));
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(!(sr & Spi::RXNE_MASK));
    sc_assert(!(sr & Spi::TXE_MASK));
    sc_assert(test.irq.read() == 0);

    wait(sc_time(32, SC_US));  // Start send byte (3/4)
    sc_assert(test.checkPayload(0x0001));
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(test.irq.read() == 1);

    wait(sc_time(32, SC_US));  // Start send byte (4/4)
    sc_assert(test.checkPayload(0x00ef));
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(test.irq.read() == 1);

    wait(sc_time(32, SC_US));  // Finish send byte (4/4)
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(!(sr & Spi::BSY_MASK));
    sc_assert(test.irq.read() == 1);

    // Empty rx register
    val = read16(OFS_SPI_DR);
    sc_assert(val == 0xcdcd);
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    val = read16(OFS_SPI_DR);
    sc_assert(val == 0xcdcd);
    sr = read16(OFS_SPI_SR);
    sc_assert(!(sr & Spi::RXNE_MASK));
    sc_assert(test.irq.read() == 1);

    // Clear interrupt request
    test.active_exception.write(SPI1_EXCEPT_ID + 16);
    wait(sc_time(1, SC_US));
    sc_assert(test.irq.read() == 0);
    test.active_exception.write(-1);

    spdlog::info("SUCCESS");

    // ------ TEST: RXNE interrupt
    spdlog::info("Testing RXNE interrupt...");
    test.m_dut.reset();
    write16(OFS_SPI_CR2,
            Spi::RXNEIE_MASK |           // RXNE interrupt enable
                (7u << Spi::DS_SHIFT));  // 8-bit data size
    write16(OFS_SPI_CR1,
            (1u << Spi::BR_SHIFT) |        // Baudrate = clk/4
                (1u << Spi::SPE_SHIFT) |   // Enable
                (1u << Spi::MSTR_SHIFT));  // Master mode
    write16(OFS_SPI_DR, 0xabcd);           // Write data -> trigger transaction
    write16(OFS_SPI_DR, 0xef01);           // Fill tx buffer

    wait(sc_time(1, SC_US));  // Start send byte (1/4)
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(!(sr & Spi::TXE_MASK));
    sc_assert(!(sr & Spi::RXNE_MASK));
    sc_assert(test.checkPayload(0x00cd));
    sc_assert(test.irq.read() == 0);

    wait(sc_time(32, SC_US));  // Start send byte (2/4)
    sc_assert(test.checkPayload(0x00ab));
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(!(sr & Spi::RXNE_MASK));
    sc_assert(!(sr & Spi::TXE_MASK));
    sc_assert(test.irq.read() == 0);

    wait(sc_time(32, SC_US));  // Start send byte (3/4)
    sc_assert(test.checkPayload(0x0001));
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(test.irq.read() == 1);

    wait(sc_time(32, SC_US));  // Start send byte (4/4)
    sc_assert(test.checkPayload(0x00ef));
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(sr & Spi::BSY_MASK);
    sc_assert(test.irq.read() == 1);

    wait(sc_time(32, SC_US));  // Finish send byte (4/4)
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    sc_assert(sr & Spi::TXE_MASK);
    sc_assert(!(sr & Spi::BSY_MASK));
    sc_assert(test.irq.read() == 1);

    // Empty rx register
    val = read16(OFS_SPI_DR);
    sc_assert(val == 0xcdcd);
    sr = read16(OFS_SPI_SR);
    sc_assert(sr & Spi::RXNE_MASK);
    val = read16(OFS_SPI_DR);
    sc_assert(val == 0xcdcd);
    sr = read16(OFS_SPI_SR);
    sc_assert(!(sr & Spi::RXNE_MASK));
    sc_assert(test.irq.read() == 1);

    // Clear interrupt request
    test.active_exception.write(SPI1_EXCEPT_ID + 16);
    wait(sc_time(1, SC_US));
    sc_assert(test.irq.read() == 0);
    test.active_exception.write(-1);

    spdlog::info("SUCCESS");

    spdlog::info("TEST SUITE SUCCESSFUL");

    sc_stop();
  }
  void write16(const uint16_t addr, const uint16_t val, bool doWait = true) {
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

  uint16_t read16(const uint16_t addr, bool doWait = true) {
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
