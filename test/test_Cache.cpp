/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <algorithm>
#include <array>
#include <random>
#include <string>
#include <systemc>
#include <tlm>
#include "include/fused.h"
#include "mcu/Cache.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/NonvolatileMemory.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "ps/PowerModelChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
 public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood", false};
  sc_signal<unsigned> framWaitStates{"framWaitStates", 1};
  tlm_utils::simple_initiator_socket<dut> cacheSocket{"cacheSocket"};
  ClockSourceChannel clk{"clk", sc_time(1, SC_NS)};
  PowerModelChannel powerModelChannel{
      "powerModelChannel", "/tmp",
      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.systemClk.bind(clk);
    nvm.pwrOn.bind(pwrGood);
    nvm.systemClk.bind(clk);
    m_dut.tSocket.bind(cacheSocket);
    m_dut.iSocket.bind(nvm.tSocket);
    nvm.waitStates.bind(framWaitStates);
    m_dut.powerModelPort.bind(powerModelChannel);
    nvm.powerModelPort.bind(powerModelChannel);
  }

  Cache m_dut{"dut", NVRAM_START, NVRAM_START + NVRAM_SIZE - 1};
  NonvolatileMemory nvm{"nvm", NVRAM_START, NVRAM_START + NVRAM_SIZE - 1};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) {
    NVM_SIZE = test.nvm.size();
    sc_assert(test.nvm.size() ==
              test.m_dut.endAddress() - test.m_dut.startAddress() + 1);
    SC_THREAD(runtests);
  }

  void runtests() {
    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // ------ TEST: Basic write & read
    for (int addr = 0; addr < NVM_SIZE; addr += TARGET_WORD_SIZE) {
      writeWord(test.cacheSocket, addr, 0xABBA);
    }
    for (int addr = 0; addr < NVM_SIZE; addr += TARGET_WORD_SIZE) {
      sc_assert(readWord(test.cacheSocket, addr) == 0xABBA);
    }

    // ------ TEST: Write & read back random values in random order
    // Generate random values
    std::vector<unsigned> values(NVM_SIZE / TARGET_WORD_SIZE);
    std::generate(values.begin(), values.end(), std::rand);

    // Generate random sequence of all valid addresses
    std::vector<unsigned> addresses;
    for (unsigned i = 0; i < NVM_SIZE / TARGET_WORD_SIZE; i++) {
      addresses.push_back(i * TARGET_WORD_SIZE);
    }
    std::random_shuffle(addresses.begin(), addresses.end());

    // Write
    for (int i = 0; i < addresses.size(); i++) {
      writeWord(test.cacheSocket, addresses[i], values[i]);
    }

    // Read
    for (int i = 0; i < addresses.size(); i++) {
      unsigned mask = (1ull << 8 * TARGET_WORD_SIZE) - 1;
      sc_assert(readWord(test.cacheSocket, addresses[i]) == (values[i] & mask));
    }

    spdlog::info("Test successful.");
    sc_stop();
  }

  void writeWord(tlm_utils::simple_initiator_socket<dut> & socket,
                 const uint32_t addr, const uint32_t val, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[TARGET_WORD_SIZE];
    trans.set_data_ptr(data);
    trans.set_data_length(TARGET_WORD_SIZE);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);

    if (TARGET_WORD_SIZE == 4) {
      Utility::unpackBytes(data, Utility::htotl(val), 4);
    } else if (TARGET_WORD_SIZE == 2) {
      Utility::unpackBytes(data, Utility::htots(val), 2);
    } else {
      SC_REPORT_FATAL(this->name(), "Invalid TARGET_WORD_SIZE");
    }

    socket->b_transport(trans, delay);
    if (doWait) {
      wait(delay);
    }
  }

  uint32_t readWord(tlm_utils::simple_initiator_socket<dut> & socket,
                    const uint32_t addr, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[TARGET_WORD_SIZE];
    trans.set_data_ptr(data);
    trans.set_data_length(TARGET_WORD_SIZE);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);
    socket->b_transport(trans, delay);

    if (doWait) {
      wait(delay);
    }

    if (TARGET_WORD_SIZE == 4) {
      return Utility::ttohl(Utility::packBytes(data, 4));
    } else if (TARGET_WORD_SIZE == 2) {
      return Utility::ttohs(Utility::packBytes(data, 2));
    } else {
      SC_REPORT_FATAL(this->name(), "Invalid TARGET_WORD_SIZE");
      return -1;
    }
  }

  // Vars
  unsigned NVM_SIZE;
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
