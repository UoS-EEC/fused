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
#include "mcu/GenericMemory.hpp"
#include "mcu/msp430fr5xx/Msp430Cpu.hpp"
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
  sc_signal<bool> stallCpu{"stallCpu"};
  sc_signal<unsigned> irqIdx{"irqIdx"};
  sc_signal<bool> iraConnected{"iraConnected"};
  GenericMemory mem{"mem", 0, 0xFFFF, sc_time(125, SC_NS)};  //! 65k memory

  SC_CTOR(dut) {
    mem.pwrOn.bind(nreset);
    mem.tSocket.bind(m_dut.iSocket);
    m_dut.pwrOn.bind(nreset);
    m_dut.irq.bind(irq);
    m_dut.ira.bind(ira);
    m_dut.irqIdx.bind(irqIdx);
    m_dut.iraConnected.bind(iraConnected);
    m_dut.busStall.bind(stallCpu);
  }

  Msp430Cpu m_dut{"dut", sc_time(125, SC_NS)};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  /*
   * Run instructions one by one & check results and timing. A NOP should be
   * inserted between each instruction.
   */
  void runtests() {
    test.m_dut.reset();
    // test.m_dut.unstall();
    test.nreset.write(true);
    wait(SC_ZERO_TIME);
    test.m_dut.dbg_writeReg(SR_REGNUM, 0x00);  // Clear CPUOFF flag

    // TEST -- AND #imm, rn
    spdlog::info("TEST: AND #255, r12");
    test.m_dut.dbg_writeReg(12, 0xAAAA);
    std::cout << test.m_dut;
    writeMemory16(0, 0xf03c);  // AND #imm, r12
    writeMemory16(2, 0x00ff);  // imm=255
    writeMemory16(4, 0x4303);  // NOP

    test.m_dut.unstall();
    wait(2 * test.m_dut.m_cycleTime);
    wait(SC_ZERO_TIME);
    std::cout << test.m_dut;
    sc_assert(test.m_dut.dbg_readReg(12) == 0xaa);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 4);

    // TEST -- MOV.B rs, rd
    spdlog::info("TEST: MOV.b r12, r13");
    test.m_dut.reset();
    test.m_dut.dbg_writeReg(SR_REGNUM, 0x00);  // Clear CPUOFF flag
    test.m_dut.dbg_writeReg(12, 0xabcd);       // Set source value
    writeMemory16(2, 0x4c4d);                  // MOV.B #ofs(r1), r12
    writeMemory16(4, 0x4303);                  // NOP
    std::cout << test.m_dut;

    wait(1 * test.m_dut.m_cycleTime);  // Execute NOP
    wait(1 * test.m_dut.m_cycleTime);  // Execute MOV
    wait(SC_ZERO_TIME);
    std::cout << test.m_dut;
    sc_assert(test.m_dut.dbg_readReg(13) == 0xcd);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 4);

    // TEST -- MOV.B ofs(rs), rd
    spdlog::info("TEST: MOV.B 10(r1), r12");
    test.m_dut.reset();
    test.m_dut.dbg_writeReg(SR_REGNUM, 0x00);  // Clear CPUOFF flag
    writeMemory16(2, 0x415c);                  // MOV.B #ofs(r1), r12
    writeMemory16(4, 0x000a);                  // #ofs = 10
    writeMemory16(6, 0x4303);                  // NOP
    writeMemory16(0xa, 0xabcd);                // val to load = 0xabcd
    std::cout << test.m_dut;

    wait(1 * test.m_dut.m_cycleTime);  // Execute NOP
    wait(3 * test.m_dut.m_cycleTime);  // Execute MOV.B
    wait(SC_ZERO_TIME);
    std::cout << test.m_dut;
    sc_assert(test.m_dut.dbg_readReg(12) == 0x00cd);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 6);

    spdlog::info("Msp430Cpu tests PASSED");
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

  static const unsigned PC_REGNUM = 0;
  static const unsigned SP_REGNUM = 1;
  static const unsigned SR_REGNUM = 2;
  static const unsigned CG_REGNUM = 3;
  static const unsigned N_GPR = 16;  // How many general purpose registers

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
