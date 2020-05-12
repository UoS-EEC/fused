/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/GenericMemory.hpp"
#include "mcu/msp430fr5xx/Dma.hpp"
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
  sc_signal<bool> nreset{"nreset", false};
  sc_signal<bool> irq{"irq"};
  sc_signal<bool> ira{"ira"};
  std::array<sc_signal<bool>, 30> trigger;
  GenericMemory mem{"mem", 0, 0xFFFF, sc_time(1, SC_US)};  //! 65k memory
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel clk{"clk", sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(nreset);
    mem.pwrOn.bind(nreset);
    m_dut.tSocket.bind(iSocket);
    mem.tSocket.bind(m_dut.iSocket);
    m_dut.clk.bind(clk);
    m_dut.irq.bind(irq);
    m_dut.ira.bind(ira);
    for (auto i = 0; i < trigger.size(); i++) {
      m_dut.trigger[i].bind(trigger[i]);
    }
  }

  Dma m_dut{"dut", sc_time(1, SC_NS)};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    test.nreset.write(true);
    wait(SC_ZERO_TIME);

    // TEST -- Basic transfer
    //   - Transfermode Single,
    //   - increment source and destination address
    //   - size: 32 byte
    //   - word size: 16-bit

    // Set memory contents
    for (auto i = 0; i < 1024; i++) {
      writeMemory16(2 * i, i);
    }

    // Configure DMA
    write16(OFS_DMA0SA, 0);
    write16(OFS_DMA0DA, 128);
    write16(OFS_DMA0SZ, 32);
    write16(OFS_DMA0CTL, DMADT_0 | DMADSTINCR_3 | DMASRCINCR_3 |
                             DMADSTBYTE__WORD | DMASRCBYTE__WORD |
                             DMALEVEL__EDGE | DMAEN_1 | DMAIE);
    // Trigger
    for (auto i = 0; i < 32; i++) {
      std::cout << *test.m_dut.m_channels[0] << std::endl;
      test.trigger[0].write(true);
      wait(1 * test.clk.getPeriod());
      test.trigger[0].write(false);
      wait(4 * test.clk.getPeriod());
    }
    std::cout << *test.m_dut.m_channels[0] << std::endl;

    // Should be disabled after completed transfer
    sc_assert(!test.m_dut.m_channels[0]->enable);
    sc_assert(!(read16(OFS_DMA0CTL) & DMAEN));

    // Check interrupt flag
    sc_assert(test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(test.irq.read());
    sc_assert(read16(OFS_DMAIV) == 2u);  // Clears irq
    wait(test.clk.getPeriod());
    auto tmp = read16(OFS_DMAIV);
    std::cout << "DMAIV: " << tmp << std::endl;
    // sc_assert(read16(OFS_DMAIV) == 0);

    sc_assert(!test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(!test.irq.read());

    // Check memory contents
    for (auto i = 0; i < 31; i++) {
      std::cout << 2 * i + 128 << ": " << readMemory16(2 * i + 128)
                << std::endl;
      sc_assert(readMemory16(2 * i + 128) == i);
    }

    // TEST -- CSR reset value
    // sc_assert(read32(OFS_SYST_CSR) == 0x4);

    // TEST -- TENMS register
    /*
    sc_assert(
        read32(OFS_SYST_CALIB, false) ==
        (SYST_CALIB_NOREF |
         (static_cast<int>(sc_time(10, SC_MS) / test.clk.getPeriod()) - 1)));
         */
    sc_stop();
  }

  void writeMemory16(const uint32_t addr, const uint32_t val) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[2];
    trans.set_data_ptr(data);
    trans.set_data_length(2);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);

    Utility::unpackBytes(data, Utility::htots(val), 2);
    test.mem.transport_dbg(trans);  // Bypassing sockets
  }

  int readMemory16(const uint32_t addr) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[2];
    trans.set_data_ptr(data);
    trans.set_data_length(2);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);

    test.mem.transport_dbg(trans);  // Bypassing sockets
    return Utility::ttohs(Utility::packBytes(data, 2));
  }

  void write16(const uint32_t addr, const uint32_t val, bool doWait = true) {
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

  uint32_t read16(const uint32_t addr, bool doWait = true) {
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
