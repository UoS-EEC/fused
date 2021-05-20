/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "include/cm0-fused.h"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/GenericMemory.hpp"
#include "mcu/cortex-m0/Dma.hpp"
#include "ps/PowerModelChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"
#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>

// get DMA bitfield definitions from the MSP430 header
#undef DMA_BASE // Need to undef this first
extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;
using namespace Utility;
using namespace CortexM0Peripherals;

SC_MODULE(dut) {
public:
  // Signals
  sc_signal<bool> nreset{"nreset", false};
  sc_signal<bool> irq{"irq"};
  sc_signal<int> active_exception{"active_exception", -1};
  sc_signal<bool> busStall{"busStall"};
  ClockSourceChannel mclk{"mclk", sc_time(125, SC_NS)};
  std::array<sc_signal<bool>, 30> trigger;
  GenericMemory mem{"mem", 0, 0xFFFF}; //! 65k memory
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  PowerModelChannel powerModelChannel{"powerModelChannel", "/tmp",
                                      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(nreset);
    mem.pwrOn.bind(nreset);
    mem.systemClk.bind(mclk);
    m_dut.tSocket.bind(iSocket);
    mem.tSocket.bind(m_dut.iSocket);
    m_dut.irq.bind(irq);
    m_dut.active_exception.bind(active_exception);
    m_dut.systemClk.bind(mclk);
    m_dut.busStall.bind(busStall);
    for (auto i = 0; i < trigger.size(); i++) {
      m_dut.trigger[i].bind(trigger[i]);
    }
    m_dut.powerModelPort.bind(powerModelChannel);
    mem.powerModelPort.bind(powerModelChannel);
  }

  Dma m_dut{"dut", 0};
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
    //   - word size: 32 bit

    spdlog::info("Testing basic single transfer");

    // Set memory contents
    for (auto i = 0; i < 2048; i++) {
      writeMemory32(4 * i, i);
    }

    // Configure DMA
    write32(Dma::RegisterAddress::DMA0SA, 0);
    write32(Dma::RegisterAddress::DMA0DA, 128);
    write32(Dma::RegisterAddress::DMA0SZ, 32);
    write32(Dma::RegisterAddress::DMA0CTL,
            DMADT_0 | DMADSTINCR_3 | DMASRCINCR_3 | DMADSTBYTE__WORD |
                DMASRCBYTE__WORD | DMALEVEL__EDGE | DMAEN_1 | DMAIE);
    // Trigger
    for (auto i = 0; i < 32; i++) {
      // std::cout << *test.m_dut.m_channels[0] << std::endl;
      test.trigger[0].write(true);
      wait(1 * test.mclk.getPeriod());
      sc_assert(test.m_dut.m_channels[0]->enable);
      sc_assert(test.busStall.read());
      test.trigger[0].write(false);
      wait(4 * test.mclk.getPeriod());
    }
    // std::cout << *test.m_dut.m_channels[0] << std::endl;

    // Should be disabled after completed transfer
    sc_assert(!test.m_dut.m_channels[0]->enable);
    sc_assert(!(read32(Dma::RegisterAddress::DMA0CTL) & DMAEN));
    sc_assert(!test.busStall.read());

    // Check interrupt flag
    sc_assert(test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(test.irq.read());
    sc_assert(read32(Dma::RegisterAddress::DMAIV) == 2u);

    // Clear IRQ
    test.active_exception.write(DMA_EXCEPT_ID + 16);
    wait(test.mclk.getPeriod());
    test.active_exception.write(-1);
    sc_assert(read32(Dma::RegisterAddress::DMAIV) == 0);
    sc_assert(!test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(!test.irq.read());

    // Check memory contents
    for (auto i = 0; i < 31; i++) {
      // std::cout << 2 * i + 128 << ": " << readMemory16(2 * i + 128)
      //<< std::endl;
      sc_assert(readMemory32(4 * i + 128) == i);
    }

    // TEST -- Block transfer
    spdlog::info("Testing basic block transfer");

    // Configure DMA
    write32(Dma::RegisterAddress::DMA0SA, 0);
    write32(Dma::RegisterAddress::DMA0DA, 256);
    write32(Dma::RegisterAddress::DMA0SZ, 32);
    write32(Dma::RegisterAddress::DMA0CTL,
            DMADT_1 | DMADSTINCR_3 | DMASRCINCR_3 | DMADSTBYTE__WORD |
                DMASRCBYTE__WORD | DMALEVEL__EDGE | DMAEN_1 | DMAIE);
    // Trigger
    wait(test.mclk.getPeriod());
    test.trigger[0].write(true);
    wait(test.mclk.getPeriod());
    test.trigger[0].write(false);
    for (auto i = 0; i < 32; i++) {
      // std::cout << *test.m_dut.m_channels[0] << std::endl;
      sc_assert(test.m_dut.m_channels[0]->enable);
      sc_assert(test.busStall.read());
      wait(2 * test.mclk.getPeriod());
    }
    wait(test.mclk.getPeriod());
    // std::cout << *test.m_dut.m_channels[0] << std::endl;
    sc_assert(!test.m_dut.m_channels[0]->enable);
    sc_assert(!test.busStall.read());

    // Check interrupt flag
    sc_assert(test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(test.irq.read());
    sc_assert(read32(Dma::RegisterAddress::DMAIV) == 2u);
    test.active_exception.write(DMA_EXCEPT_ID + 16);
    wait(test.mclk.getPeriod());
    test.active_exception.write(-1);
    sc_assert(read32(Dma::RegisterAddress::DMAIV) == 0);
    sc_assert(!test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(!test.irq.read());

    // Check memory contents
    for (auto i = 0; i < 31; i++) {
      // std::cout << 2 * i + 256 << ": " << readMemory16(2 * i + 256)
      //<< std::endl;
      sc_assert(readMemory32(4 * i + 256) == i);
    }

    // TEST -- Software trigger
    //   - Transfermode Single,
    //   - increment source and destination address
    //   - size: 32 byte
    //   - word size: 32 bit
    spdlog::info("Testing software trigger");

    // Configure DMA
    write32(Dma::RegisterAddress::DMA0SA, 0);
    write32(Dma::RegisterAddress::DMA0DA, 128);
    write32(Dma::RegisterAddress::DMA0SZ, 32);
    unsigned ctrl = DMADT_0 | DMADSTINCR_3 | DMASRCINCR_3 | DMADSTBYTE__WORD |
                    DMASRCBYTE__WORD | DMALEVEL__EDGE | DMAEN_1 | DMAIE;

    write32(Dma::RegisterAddress::DMA0CTL, ctrl); // Trigger
    for (auto i = 0; i < 32; i++) {
      // std::cout << *test.m_dut.m_channels[0] << std::endl;
      write32(Dma::RegisterAddress::DMA0CTL, ctrl | DMAREQ);
      sc_assert(test.m_dut.m_channels[0]->enable);
      sc_assert(test.busStall.read());
      wait(3 * test.mclk.getPeriod());
    }
    // std::cout << *test.m_dut.m_channels[0] << std::endl;

    // Should be disabled after completed transfer
    sc_assert(!test.m_dut.m_channels[0]->enable);
    sc_assert(!test.busStall.read());
    sc_assert(!(read32(Dma::RegisterAddress::DMA0CTL) & DMAEN));

    // Check interrupt flag
    sc_assert(test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(test.irq.read());
    sc_assert(read32(Dma::RegisterAddress::DMAIV) == 2u);
    test.active_exception.write(DMA_EXCEPT_ID + 16);
    wait(test.mclk.getPeriod());
    test.active_exception.write(-1);
    sc_assert(read32(Dma::RegisterAddress::DMAIV) == 0);
    sc_assert(!test.m_dut.m_channels[0]->interruptFlag);
    sc_assert(!test.irq.read());

    sc_stop();
  }

  void writeMemory32(const uint32_t addr, const uint32_t val) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[4];
    trans.set_data_ptr(data);
    trans.set_data_length(4);
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);

    Utility::unpackBytes(data, Utility::htotl(val), 4);
    test.mem.transport_dbg(trans); // Bypassing sockets
  }

  int readMemory32(const uint32_t addr) {
    sc_time delay = SC_ZERO_TIME;
    tlm::tlm_generic_payload trans;
    unsigned char data[4];
    trans.set_data_ptr(data);
    trans.set_data_length(4);
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);

    test.mem.transport_dbg(trans); // Bypassing sockets
    return Utility::ttohl(Utility::packBytes(data, 4));
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
  auto &config = Config::get();
  config.parseFile();

  tester t("tester");
  sc_start();
  return false;
}
