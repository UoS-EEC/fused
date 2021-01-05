/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <array>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/cortex-m0/Nvic.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "ps/PowerModelChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

using namespace sc_core;
using namespace Utility;

SC_MODULE(dut) {
 public:
  // Signals
  sc_signal<bool> pwrGood{"pwrGood"};
  sc_signal<int> pending{"pending"};
  sc_signal<int> returning{"returning", -1};
  sc_signal<int> active{"active", -1};
  std::array<sc_signal<bool>, 32> irq;
  tlm_utils::simple_initiator_socket<dut> iSocket{"iSocket"};
  ClockSourceChannel clk{"clk", sc_time(1, SC_US)};
  PowerModelChannel powerModelChannel{
      "powerModelChannel", "/tmp/testPowerModelChannel.csv",
      sc_time(1, SC_US)};

  SC_CTOR(dut) {
    m_dut.pwrOn.bind(pwrGood);
    m_dut.systemClk.bind(clk);
    m_dut.tSocket.bind(iSocket);
    for (unsigned i = 0; i < irq.size(); i++) {
      m_dut.irq[i].bind(irq[i]);
      irq[i].write(false);
    }
    m_dut.pending.bind(pending);
    m_dut.returning.bind(returning);
    m_dut.active.bind(active);
    m_dut.powerModelPort.bind(powerModelChannel);
  }

  Nvic m_dut{"dut"};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  void runtests() {
    test.pwrGood.write(true);
    wait(SC_ZERO_TIME);

    // ------ TEST: ISER Write-one-to-set
    test.m_dut.reset();
    write32(OFS_NVIC_ISER, 0xAAAAAAAA, false);
    write32(OFS_NVIC_ISER, 0x00000000, false);  // Should have no effect
    sc_assert(read32(OFS_NVIC_ISER) == 0xAAAAAAAA);
    sc_assert(read32(OFS_NVIC_ICER) == 0xAAAAAAAA);

    // ------ TEST: ICER Write-one-to-clear
    test.m_dut.reset();
    // set all 1s
    write32(OFS_NVIC_ISER, 0xFFFFFFFF, false);

    // Clear to 5's
    write32(OFS_NVIC_ICER, 0xAAAAAAAA, false);
    write32(OFS_NVIC_ICER, 0x00000000, false);  // Should have no effect
    sc_assert(read32(OFS_NVIC_ISER) == 0x55555555);
    sc_assert(read32(OFS_NVIC_ICER) == 0x55555555);

    // ------ TEST: IPRx regs
    // Only two top bits of each byte should be writeable/readable
    test.m_dut.reset();
    for (unsigned i = 0; i < 8; i++) {
      test.m_dut.reset();
      write32(OFS_NVIC_IPR0 + 4 * i, 0xFFFFFFFF, false);
      sc_assert(read32(OFS_NVIC_IPR0 + 4 * i) == 0xc0c0c0c0);
    }

    // ------ TEST: Enable & Level-sensitive IRQ
    test.m_dut.reset();
    test.irq[3].write(true);
    write32(OFS_NVIC_ISER, ~(1u << 3), false);  // Wrong enable bits
    wait(sc_time(3, SC_US));
    sc_assert(test.m_dut.pending.read() == -1);
    write32(OFS_NVIC_ISER, (1u << 3), false);  // Correct enable bit
    wait(sc_time(3, SC_US));
    sc_assert(test.m_dut.pending.read() == 3 + NVIC_EXCEPT_ID_BASE);
    write32(OFS_NVIC_ICER, (1u << 3),
            false);  // Disabling should not clear pending
    sc_assert(test.m_dut.pending.read() == 3 + NVIC_EXCEPT_ID_BASE);

    // ------ TEST: Pulsed IRQ
    test.m_dut.reset();
    write32(OFS_NVIC_ISER, (1u << 3), false);  // Correct enable bit
    test.irq[3].write(true);
    wait(sc_time(1, SC_US));
    test.irq[3].write(false);
    wait(sc_time(1, SC_US));
    sc_assert(test.m_dut.pending.read() == 3 + NVIC_EXCEPT_ID_BASE);

    // ------ TEST: Software-set IRQ
    test.m_dut.reset();
    write32(OFS_NVIC_ISER, (1u << 4), false);  // Correct enable bit
    wait(sc_time(2, SC_US));
    sc_assert(test.m_dut.pending.read() == -1);
    write32(OFS_NVIC_ISPR, ~(1u << 4), false);  // Wrong bits
    wait(sc_time(1, SC_US));
    sc_assert(test.m_dut.pending.read() == -1);
    write32(OFS_NVIC_ISPR, (1u << 4), false);  // Correct bit
    wait(sc_time(2, SC_US));
    sc_assert(test.m_dut.pending.read() == 4 + NVIC_EXCEPT_ID_BASE);

    // ------ TEST: Software-clear pending IRQ
    test.m_dut.reset();
    test.irq[3].write(true);
    write32(OFS_NVIC_ISER, (1u << 3), false);  // Correct enable bit
    wait(sc_time(1, SC_US));
    sc_assert(test.m_dut.pending.read() == 3 + NVIC_EXCEPT_ID_BASE);
    wait(sc_time(1, SC_US));
    sc_assert(test.m_dut.pending.read() ==
              3 + NVIC_EXCEPT_ID_BASE);  // Should still be active (latched)
    write32(OFS_NVIC_ICPR, (1u << 3), false);  // Clear pending register
    wait(sc_time(1, SC_US));
    sc_assert(
        test.m_dut.pending.read() ==
        3 + NVIC_EXCEPT_ID_BASE);  // Should still be active (irq still high)
    test.irq[3].write(false);
    write32(OFS_NVIC_ICPR, (1u << 3), false);  // Clear pending register
    wait(sc_time(1, SC_US));
    sc_assert(test.m_dut.pending.read() == -1);  // Should be cleared

    // ------ TEST: Prioritized sequence of IRQs
    test.m_dut.reset();
    write32(OFS_NVIC_IPR0, 0xc0c0c0c0, false);  // irq[3:0] prio 3
    write32(OFS_NVIC_IPR1, 0x80808080, false);  // irq[7:4] prio 2
    write32(OFS_NVIC_IPR2, 0x40404040, false);  // irq[11:8] prio 1
    write32(OFS_NVIC_IPR3, 0x00000000, false);  // irq[15:12] prio 0
    write32(OFS_NVIC_ISER, 0xffff, false);
    test.irq[0].write(true);
    test.irq[4].write(true);
    test.irq[8].write(true);
    test.irq[12].write(true);
    wait(sc_time(1, SC_US));
    test.irq[0].write(false);
    test.irq[4].write(false);
    test.irq[8].write(false);
    test.irq[12].write(false);

    // Test & clear registers in priority order
    sc_assert(test.m_dut.pending.read() == 12 + NVIC_EXCEPT_ID_BASE);
    write32(OFS_NVIC_ICPR, (1u << 12), false);
    wait(sc_time(1, SC_US));
    sc_assert(test.m_dut.pending.read() == 8 + NVIC_EXCEPT_ID_BASE);
    write32(OFS_NVIC_ICPR, (1u << 8), false);
    wait(sc_time(2, SC_US));
    sc_assert(test.m_dut.pending.read() == 4 + NVIC_EXCEPT_ID_BASE);
    write32(OFS_NVIC_ICPR, (1u << 4), false);
    wait(sc_time(2, SC_US));
    sc_assert(test.m_dut.pending.read() == 0 + NVIC_EXCEPT_ID_BASE);
    write32(OFS_NVIC_ICPR, (1u << 0), false);
    wait(sc_time(2, SC_US));
    sc_assert(test.m_dut.pending.read() == -1);

    // ------ TEST: Accept an interrupt
    test.m_dut.reset();
    write32(OFS_NVIC_ISER, (1u << 3), false);  // Correct enable bit
    test.irq[3].write(true);
    wait(sc_time(1, SC_US));
    test.irq[3].write(false);
    wait(sc_time(1, SC_US));
    sc_assert(test.pending.read() == 3 + NVIC_EXCEPT_ID_BASE);
    test.active.write(3 + NVIC_EXCEPT_ID_BASE);
    wait(sc_time(1, SC_US));
    sc_assert(test.pending.read() == -1);
    // Reset
    test.active.write(-1);
    test.returning.write(-1);

    // ------ TEST: Interrupt still requesting during exception return
    test.m_dut.reset();
    write32(OFS_NVIC_ISER, (1u << 3), false);  // Correct enable bit
    test.irq[3].write(true);
    wait(sc_time(1, SC_US));
    sc_assert(test.pending.read() == 3 + NVIC_EXCEPT_ID_BASE);
    test.active.write(3 + NVIC_EXCEPT_ID_BASE);
    wait(sc_time(1, SC_US));
    sc_assert(test.pending.read() == -1);
    test.returning.write(3 + NVIC_EXCEPT_ID_BASE);
    wait(sc_time(1, SC_US));
    sc_assert(test.pending.read() == 3 + NVIC_EXCEPT_ID_BASE);
    // Reset
    test.active.write(-1);
    test.returning.write(-1);
    test.irq[3].write(false);

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
