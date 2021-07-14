/*
 * Copyright (c) 2018-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#include "include/memic-fused.h"
#include "mcu/CacheController.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "ps/PowerModelChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"
#include <array>
#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood", false};
  sc_signal<bool> doFlush{"doFlush"};
  sc_signal<int> nDirtyLines{"nDirtyLines", 0};
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel clk{"clk", sc_time(1, SC_NS)};
  PowerModelChannel powerModelChannel{"powerModelChannel", "/tmp",
                                      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.powerModelPort.bind(powerModelChannel);
    m_dut.systemClk.bind(clk);
    m_dut.tSocket.bind(iSocket);
    m_dut.doFlush.bind(doFlush);
    m_dut.nDirtyLines.bind(nDirtyLines);
  }

  CacheController m_dut{"dut", DCACHE_CTRL_BASE};
};

SC_MODULE(tester) {
public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // ------ TEST: Reset values
    std::cerr << "Testing: Reset values ... ";
    sc_assert(read32(OFS_DCACHE_CTRL_CSR) == 0);
    sc_assert(read32(OFS_DCACHE_CTRL_CRNTDIRTY) == 0);
    sc_assert(read32(OFS_DCACHE_CTRL_MAXDIRTY) == 0xffffffff);
    std::cerr << "OK" << std::endl;

    // ------ TEST: Flush command ignored when no dirty lines
    std::cerr << "Testing: Flush command with no dirty lines... ";
    test.nDirtyLines.write(0);
    write32(OFS_DCACHE_CTRL_CSR, DCACHE_CTRL_FLUSH);
    wait(sc_time(1, SC_US));
    sc_assert(test.doFlush.read() ==
              false); // No dirty lines, flush command ignored
    sc_assert(read32(OFS_DCACHE_CTRL_CSR) == 0);
    std::cerr << "OK" << std::endl;

    // ------ TEST: Flush command works when there are dirty lines
    std::cerr << "Testing: Flush command with dirty lines ... ";
    test.m_dut.reset();
    for (int i = 4; i > 0; i--) {
      test.nDirtyLines.write(i);
      write32(OFS_DCACHE_CTRL_CSR, DCACHE_CTRL_FLUSH);
      wait(sc_time(1, SC_US));
      sc_assert(test.doFlush.read() == true);
      sc_assert(read32(OFS_DCACHE_CTRL_CSR) == DCACHE_CTRL_FLUSH);
      sc_assert(read32(OFS_DCACHE_CTRL_CRNTDIRTY) == i);
    }
    test.nDirtyLines.write(0);
    wait(sc_time(1, SC_US));
    // Should be done flushing
    sc_assert(test.doFlush.read() == false);
    sc_assert(read32(OFS_DCACHE_CTRL_CSR) == 0);
    sc_assert(read32(OFS_DCACHE_CTRL_CRNTDIRTY) == 0);
    std::cerr << "OK" << std::endl;

    // ------ TEST: Autoflush when too many dirty lines
    std::cerr << "Testing: Autoflush when too many dirty lines... ";
    test.m_dut.reset();
    write32(OFS_DCACHE_CTRL_MAXDIRTY, 5);
    for (int i = 10; i >= 5; i--) {
      test.nDirtyLines.write(i);
      wait(sc_time(1, SC_US));
      sc_assert(test.doFlush.read() == true);
      sc_assert(read32(OFS_DCACHE_CTRL_CRNTDIRTY) == i);
    }
    test.nDirtyLines.write(4);
    wait(sc_time(1, SC_US));
    // Should be done flushing
    sc_assert(test.doFlush.read() == false);
    sc_assert(read32(OFS_DCACHE_CTRL_CRNTDIRTY) == 4);
    std::cerr << "OK" << std::endl;

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
