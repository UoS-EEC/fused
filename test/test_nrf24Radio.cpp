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
#include "ps/PowerModelChannel.hpp"
#include "sd/Nrf24Radio.hpp"
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
  sc_signal<bool> nReset{"nReset"};                         // Active low
  sc_signal_resolved chipSelect{"chipSelect"};              // Active low
  sc_signal_resolved chipEnable{"chipEnable"};              // Active high
  sc_signal_resolved interruptRequest{"interruptRequest"};  // Active low
  // Sockets
  tlm_utils::simple_initiator_socket<dut> iSpiSocket{"iSpiSocket"};
  PowerModelChannel powerModelChannel{"powerModelChannel", "/tmp",
                                      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.nReset.bind(nReset);
    m_dut.chipSelect.bind(chipSelect);
    m_dut.chipEnable.bind(chipEnable);
    m_dut.interruptRequest.bind(interruptRequest);
    m_dut.tSocket.bind(iSpiSocket);
    m_dut.powerModelPort.bind(powerModelChannel);
  }

  Nrf24Radio m_dut{"Nrf24Radio"};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    wait(sc_time(1, SC_US));
    // Initialise
    test.nReset.write(true);
    wait(sc_time(100, SC_MS));
    test.chipSelect.write(sc_dt::sc_logic(true));
    test.chipEnable.write(sc_dt::sc_logic(false));
    test.interruptRequest.write(sc_dt::sc_logic('Z'));

    // Prepare payload object
    uint8_t data = 0xab;
    tlm::tlm_generic_payload trans;
    auto *spiExtension = new SpiTransactionExtension();

    trans.set_extension(spiExtension);
    trans.set_address(0);      // SPI doesn't use address
    trans.set_data_length(1);  // Transfer size up to 1 byte

    spiExtension->clkPeriod = sc_core::sc_time(10, sc_core::SC_US);
    spiExtension->nDataBits = 8;
    spiExtension->phase = SpiTransactionExtension::SpiPhase::CAPTURE_FIRST_EDGE;
    spiExtension->polarity = SpiTransactionExtension::SpiPolarity::HIGH;
    spiExtension->bitOrder = SpiTransactionExtension::SpiBitOrder::MSB_FIRST;
    sc_time delay = spiExtension->transferTime();

    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_data_ptr(&data);
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    wait(sc_time(1, SC_US));

    spdlog::info("{:s}: Testing Starts @{:s}", this->name(),
                 sc_time_stamp().to_string());

    // ------ TEST: Read Registers
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));
    data = R_REGISTER;
    // data = 0xa1;  // This is an invalid command
    test.iSpiSocket->b_transport(trans, delay);

    wait(delay);

    // Check response status
    if (trans.is_response_error()) {
      SC_REPORT_FATAL(this->name(), "Response error");
    } else {
      sc_assert(trans.get_response_status() ==
                tlm::tlm_response_status::TLM_OK_RESPONSE);
    }

    // Returned payload should be contents of status register
    sc_assert(spiExtension->response == (RX_P_NO_2 | RX_P_NO_1 | RX_P_NO_0));

    // Expecting data bits
    sc_assert(test.m_dut.m_payloadType == Nrf24Radio::PayloadType::DATA);

    // Read First Register (NRF_CONFIG: 0x00)
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    // Check response status
    if (trans.is_response_error()) {
      SC_REPORT_FATAL(this->name(), "Response error");
    } else {
      sc_assert(trans.get_response_status() ==
                tlm::tlm_response_status::TLM_OK_RESPONSE);
    }

    //  Check return payload
    sc_assert(spiExtension->response == EN_CRC);

    // Still expecting data bits
    sc_assert(test.m_dut.m_payloadType == Nrf24Radio::PayloadType::DATA);

    // Read next register (EN_AA: 0x01)
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);
    sc_assert(spiExtension->response == 0b00111111);

    // Warning if read beyond 5 registers
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    wait(sc_time(1, SC_US));
    test.chipSelect.write(sc_dt::sc_logic(true));

    wait(sc_time(1, SC_US));

    // ------ TEST: Write to Registers, Read from Registers
    spdlog::info("------ TEST: Write to Registers, Read from Registers");
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_payloadType == Nrf24Radio::PayloadType::COMMAND);

    // Give write to NRF_CONFIG (0x00) command
    data = W_REGISTER | EN_AA;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);
    sc_assert(spiExtension->response == (RX_P_NO_2 | RX_P_NO_1 | RX_P_NO_0));

    // Expecting data
    sc_assert(test.m_dut.m_payloadType == Nrf24Radio::PayloadType::DATA);

    // Write data
    data = 0xab;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    sc_assert(test.m_dut.m_payloadType == Nrf24Radio::PayloadType::DATA);

    wait(sc_time(1, SC_US));
    test.chipSelect.write(sc_dt::sc_logic(true));

    wait(sc_time(1, SC_US));

    // Read back written data
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_payloadType == Nrf24Radio::PayloadType::COMMAND);

    data = R_REGISTER | EN_AA;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    sc_assert(test.m_dut.m_payloadType == Nrf24Radio::PayloadType::DATA);
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    // Check that is is the same as the written
    sc_assert(spiExtension->response == 0xab);

    wait(sc_time(1, SC_US));
    test.chipSelect.write(sc_dt::sc_logic(true));

    wait(sc_time(1, SC_US));

    // ------ TEST: State Machine
    spdlog::info("------ TEST: State Machine");
    test.nReset.write(false);
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::UNDEFINED);

    test.nReset.write(true);
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::POWER_ON_RESET);
    wait(sc_time(100, SC_MS));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::POWER_DOWN);

    // Write PWR_UP = 1
    // POWER_DOWN -> STANDBY1
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    data = W_REGISTER | NRF_CONFIG;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = PWR_UP;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::OSC_STARTUP);
    wait(sc_time(3, SC_MS));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::STANDBY1);

    // CE _| , PRIM_RX = 1
    // STANDBY1 -> RX_MODE
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    data = W_REGISTER | NRF_CONFIG;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = PWR_UP | PRIM_RX;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    test.chipEnable.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::RX_SETTLING);
    wait(sc_time(130, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::RX_MODE);

    wait(sc_time(1, SC_MS));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::RX_MODE);

    // CE |_
    // RX_MODE -> STANDBY1
    test.chipEnable.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::STANDBY1);

    // ------ TEST: Writing to TX Fifo
    spdlog::info("------ TEST: Writing to TX Fifo");
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    data = W_TX_PAYLOAD;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = 0xab;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    sc_assert(!test.m_dut.m_txFifo.isEmpty());

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    // ------ TEST: Popping from TX Fifo
    // then keeping CE high
    // since tx fifo empty, TX_MODE -> STANDBY2
    spdlog::info("------ TEST: Popping from TX Fifo (single tx)");
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    data = W_REGISTER | NRF_CONFIG;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = PWR_UP;  // clear PRIM_RX
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    test.chipEnable.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_SETTLING);
    wait(sc_time(130, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_MODE);

    wait(sc_time(28, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::STANDBY2);

    // Irq line active low
    sc_assert(test.interruptRequest.read().to_bool() == 0);

    wait(sc_time(1, SC_US));

    // Clear the interrupt line
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    data = W_REGISTER | NRF_STATUS;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = TX_DS | RX_P_NO_2 | RX_P_NO_1 | RX_P_NO_0;  // clear TX_DS
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    // Irq line active low
    wait(sc_time(1, SC_US));
    sc_assert(test.interruptRequest.read().to_bool() == 1);

    // ------ TEST: Writing to TX Fifo when in STANDBY2 (CE held high)
    // auto tirggers transmit
    spdlog::info(
        "------ TEST: Writing to TX Fifo when in STANDBY2 (auto trigger)");
    test.chipSelect.write(sc_dt::sc_logic(false));
    wait(sc_time(1, SC_US));

    data = W_TX_PAYLOAD;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = 0xac;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_SETTLING);
    wait(sc_time(130, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_MODE);

    test.chipEnable.write(
        sc_dt::sc_logic(false));  // So that -> STANDBY1 instead of STANDBY2

    wait(sc_time(28, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::STANDBY1);

    // ------ TEST: Sending multiple packets
    // STANDBY1 -> TX_MODE with 3 packets in Tx Fifo
    spdlog::info("------ TEST: Sending multiple packets");
    test.chipSelect.write(sc_dt::sc_logic(false));  // Packet 1
    wait(sc_time(1, SC_US));

    data = W_TX_PAYLOAD;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = 0xac;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    test.chipSelect.write(sc_dt::sc_logic(false));  // Packet 2
    wait(sc_time(1, SC_US));

    data = W_TX_PAYLOAD;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = 0xad;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    test.chipSelect.write(sc_dt::sc_logic(false));  // Packet 3
    wait(sc_time(1, SC_US));

    data = W_TX_PAYLOAD;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = 0xae;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = 0xaf;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_txFifo.isFull());
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::STANDBY1);

    test.chipSelect.write(sc_dt::sc_logic(false));  // Change Address Width
    wait(sc_time(1, SC_US));

    data = W_REGISTER | SETUP_AW;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = AW_0;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    test.chipSelect.write(
        sc_dt::sc_logic(false));  // Change transmit power & dataRate
    wait(sc_time(1, SC_US));

    data = W_REGISTER | RF_SETUP;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    data = RF_PWR_0;
    test.iSpiSocket->b_transport(trans, delay);
    wait(delay);

    test.chipSelect.write(sc_dt::sc_logic(true));
    wait(sc_time(1, SC_US));

    test.chipEnable.write(sc_dt::sc_logic(true));  // Start Transmit
    wait(sc_time(1, SC_US));

    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_SETTLING);
    wait(sc_time(130, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_MODE);

    wait(sc_time(40, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_MODE);

    wait(sc_time(40, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::TX_MODE);

    wait(sc_time(48, SC_US));
    sc_assert(test.m_dut.m_radio_state == Nrf24Radio::OpModes::STANDBY2);

    spdlog::info("{:s}: Testing Done @{:s} ", this->name(),
                 sc_time_stamp().to_string());

    sc_stop();
  }

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  // Set up paths
  // Parse CLI arguments & config file
  auto &config = Config::get();
  config.parseFile();

  tester t("tester");
  sc_start();
  return false;
}
