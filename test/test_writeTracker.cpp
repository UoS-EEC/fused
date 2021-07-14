/*
 * Copyright (c) 2019-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "include/peripheral-defines.h"
#include "mcu/ClockSourceChannel.hpp"
#include "ps/PowerModelChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"
#include "utilities/WriteTracker.hpp"
#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
public:
  // Signals & ports
  sc_signal<bool> pwrGood{"pwrGood"};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  PowerModelChannel powerModelChannel{"powerModelChannel", "/tmp",
                                      sc_time(1, SC_US)};
  ClockSourceChannel clk{"clk", sc_time(1, SC_NS)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.systemClk.bind(clk);
    m_dut.powerModelPort.bind(powerModelChannel);
    m_dut.tSocket.bind(iSocket);
  }

  WriteTracker m_dut{"dut", /*startAddress=*/0x0,
                     /*monitorEndAddress=*/0x1fff,
                     /*blockSize=*/64};
};

SC_MODULE(tester) {
public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    // Analysis port transaction
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[] = {0x55, 0x55, 0x55, 0x55};
    trans.set_data_ptr(data);
    trans.set_data_length(sizeof(data));

    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    spdlog::info("------ TEST: Write while monitoring disabled");
    test.m_dut.reset();
    sc_assert(read32(WriteTracker::RegisterAddress::Control) == 0);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(0x1000);
    test.m_dut.write(trans);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty + 0) == 0);

    spdlog::info("------ TEST: Write while monitoring enabled");
    test.m_dut.reset();
    sc_assert(read32(WriteTracker::RegisterAddress::Control) == 0);
    write32(WriteTracker::RegisterAddress::Control,
            WriteTracker::BitMask::Control_Enable);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(0x1000);
    test.m_dut.write(trans);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty + 0) == 0b1);
    trans.set_address(0x1004);
    test.m_dut.write(trans);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty + 0) == 0b1);
    trans.set_address(0x1000 + 64);
    test.m_dut.write(trans);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty + 0) == 0b11);

    spdlog::info("------ TEST: Write outside monitored space");
    test.m_dut.reset();
    sc_assert(read32(WriteTracker::RegisterAddress::Control) == 0);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty) == 0);
    write32(WriteTracker::RegisterAddress::Control,
            WriteTracker::BitMask::Control_Enable);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(0x2000);
    test.m_dut.write(trans);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty + 0) == 0b0);

    spdlog::info("------ TEST: Clear dirty bits");
    test.m_dut.reset();
    sc_assert(read32(WriteTracker::RegisterAddress::Control) == 0);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty) == 0);
    write32(WriteTracker::RegisterAddress::Control,
            WriteTracker::BitMask::Control_Enable);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(0x1000);
    test.m_dut.write(trans);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty + 0) == 0b1);
    write32(WriteTracker::RegisterAddress::Control,
            WriteTracker::BitMask::Control_Clear);
    sc_assert(read32(WriteTracker::RegisterAddress::Dirty + 0) == 0b0);

    spdlog::info("------ All tests passed ------");
    sc_stop();
  }

  void write32(const uint32_t addr, const uint32_t val, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[4];
    trans.set_data_ptr(data);
    trans.set_data_length(4);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);

    Utility::unpackBytes(data, Utility::htotl(val), 4);
    test.iSocket->b_transport(trans, delay);
    if (doWait) {
      wait(delay);
    }
  }

  uint32_t read32(const uint32_t addr, bool doWait = true) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[4];
    trans.set_data_ptr(data);
    trans.set_data_length(4);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);
    test.iSocket->b_transport(trans, delay);

    if (doWait) {
      wait(delay);
    }
    return Utility::ttohl(Utility::packBytes(data, 4));
  }

  dut test{"dut"};
};

int sc_main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  // Set up paths
  // Parse CLI arguments & config file
  Config::get().parseFile("../config/MemicBoard-config.yml");

  tester t("tester");
  sc_start();
  return false;
}
