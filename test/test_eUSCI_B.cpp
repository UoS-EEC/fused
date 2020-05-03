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
#include "mcu/SpiTransactionExtension.hpp"
#include "mcu/msp430fr5xx/eUSCI_B.hpp"
#include "ps/DynamicEnergyChannel.hpp"
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
  sc_signal<bool> ira{"ira"};
  sc_signal<bool> irq{"irq"};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  tlm_utils::simple_target_socket<dut> tSocket{"tSocket"};
  tlm_utils::simple_target_socket<dut> tEusciSocket{"tEusciSocket"};
  ClockSourceChannel smclk{"smclk", sc_time(1, SC_US)};
  ClockSourceChannel aclk{"aclk", sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.tSocket.bind(iSocket);
    m_dut.iSocket.bind(tSocket);
    m_dut.iEusciSocket.bind(tEusciSocket);
    m_dut.aclk.bind(aclk);
    m_dut.smclk.bind(smclk);
    m_dut.irq.bind(irq);
    m_dut.ira.bind(ira);

    tEusciSocket.register_b_transport(this, &dut::b_transport);
  }

  virtual void b_transport(tlm::tlm_generic_payload & trans, sc_time & delay) {
    auto *ptr = trans.get_data_ptr();
    auto len = trans.get_data_length();

    // SPI peripheral device checks if params and clock valid
    // then respond
    auto *spiExtension = trans.get_extension<SpiTransactionExtension>();
    spiExtension->response = 0x00CD;
    std::cout << *spiExtension;
    m_lastTransaction.deep_copy_from(trans);  // Copy the transaction object
    m_lastPayload = trans.get_data_ptr()[0];  // Copy payload data
    trans.set_response_status(tlm::TLM_OK_RESPONSE);
  }

  bool checkPayload(const uint8_t c) { return m_lastPayload == c; }

  // bool checkParams(uint16_t c) { return rx_spi_package.spi_parameters == c; }

  bool checkClock(const sc_time c) {
    return m_lastTransaction.get_extension<SpiTransactionExtension>()
               ->clkPeriod == c;
  }

  eUSCI_B m_dut{"dut", 0, 0x2f, sc_time(1, SC_NS)};

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
    sc_assert(read16(OFS_UCB0CTLW0) == 0x01C1);
    sc_assert(read16(OFS_UCB0BRW) == 0x0000);
    sc_assert(read16(OFS_UCB0STATW) == 0x0000);
    sc_assert(read16(OFS_UCB0RXBUF) == 0x0000);
    sc_assert(read16(OFS_UCB0TXBUF) == 0x0000);
    sc_assert(read16(OFS_UCB0IE) == 0x0000);
    sc_assert(read16(OFS_UCB0IFG) == 0x0002);
    sc_assert(read16(OFS_UCB0IV) == 0x0000);
    // Explicitly set UCSWRST bit
    write16(OFS_UCB0BRW, 0x00AA, false);
    sc_assert(read16(OFS_UCB0BRW) == 0x00AA);
    write16(OFS_UCB0CTLW0, UCSWRST, false);
    sc_assert(read16(OFS_UCB0CTLW0) == 0x01C1);
    sc_assert(read16(OFS_UCB0BRW) == 0x0000);
    sc_assert(read16(OFS_UCB0STATW) == 0x0000);
    sc_assert(read16(OFS_UCB0RXBUF) == 0x0000);
    sc_assert(read16(OFS_UCB0TXBUF) == 0x0000);
    sc_assert(read16(OFS_UCB0IE) == 0x0000);
    sc_assert(read16(OFS_UCB0IFG) == 0x0002);
    sc_assert(read16(OFS_UCB0IV) == 0x0000);

    // ------ TEST: Tx Operation
    // Writing to TXBUF clears TXIFG
    // TXIFG sets when Tx done
    write16(OFS_UCB0TXBUF, 0x00AB, true);
    // Payload transport via eusciSocket
    sc_assert(test.checkPayload(0x00AB));
    // TXIFG sets when Tx done (buffer empty)
    // RXIFG sets when Rx done (buffer full)
    sc_assert(read16(OFS_UCB0IFG) == (UCTXIFG | UCRXIFG));
    // Target reply in RXBUF
    sc_assert(read16(OFS_UCB0RXBUF) == 0x00CD);
    // Reading from RXBUF clears RFIFG
    sc_assert(read16(OFS_UCB0IFG) == UCTXIFG);

    // ------ TEST: SPI Formatted Packet
    // Reset
    write16(OFS_UCB0CTLW0, UCSWRST, false);
    // Configure SPI parameters
    // set phase, active high, 8-bit, master, 3-pin, aclk
    write16(OFS_UCB0CTLW0, UCCKPH | UCCKPL | UCMST | UCSSEL0);
    // Configure bit rate = aclk/10
    write16(OFS_UCB0BRW, 0x000a, false);
    // TX with SPI packet
    write16(OFS_UCB0TXBUF, 0x00AC, true);
    // Checked if received packet correct
    sc_assert(test.checkPayload(0x00AC));
    // sc_assert(test.checkParams(UCCKPH | UCCKPL | UCMST | UCSSEL0));
    auto ext = test.m_lastTransaction.get_extension<SpiTransactionExtension>();
    sc_assert(ext->phase ==
              SpiTransactionExtension::SpiPhase::CAPTURE_SECOND_EDGE);
    sc_assert(ext->polarity == SpiTransactionExtension::SpiPolarity::LOW);
    sc_assert(test.checkClock(sc_time(10, SC_US)));

    wait(sc_time(80, SC_US));

    // ------ TEST: SPI Operation Timing
    // Reset
    write16(OFS_UCB0CTLW0, UCSWRST, false);
    // Configure SPI parameters
    // set phase, active high, 8-bit, master, 3-pin, aclk
    write16(OFS_UCB0CTLW0, UCCKPH | UCCKPL | UCMST | UCSSEL0);
    // Configure bit rate = aclk/10
    write16(OFS_UCB0BRW, 0x000a, false);
    // Enable interrupts
    write16(OFS_UCB0IE, UCTXIE | UCRXIE);
    // Tx with SPI packet
    sc_assert((read16(OFS_UCB0STATW) & UCBUSY) == 0x00); // eUSCI not busy
    write16(OFS_UCB0TXBUF, 0x00AD, true);
    sc_assert(read16(OFS_UCB0IFG) == 0x0000);
    sc_assert((read16(OFS_UCB0STATW) & UCBUSY) == 0x01); // eUSCI busy
    wait(test.irq.posedge_event());
    std::cout << "Responding to irq" << std::endl;
    test.ira.write(1);
    std::cout << "ira done" << std::endl;
    wait(test.irq.negedge_event());
    test.ira.write(0);
    wait(test.irq.posedge_event());
    test.ira.write(1);
    std::cout << "Responding to irq" << std::endl;
    std::cout << "ira done" << std::endl;
    sc_assert(read16(OFS_UCB0IFG) == (UCTXIFG | UCRXIFG));       
    std::cout << "Checking UCBUSY @ " << sc_time_stamp() << std::endl;
    sc_assert((read16(OFS_UCB0STATW) & UCBUSY) == 0x00); // eUSCI not busy

    std::cout << std::endl << "TESTING DONE" << std::endl;
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
