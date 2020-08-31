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

  void runtests() {
    test.m_dut.dbg_writeReg(SR_REGNUM, 0);
    test.m_dut.dbg_writeReg(PC_REGNUM, 0);
    test.m_dut.unstall();
    test.nreset.write(true);
    wait(SC_ZERO_TIME);

    // Place instructions in mem for CPU to execute
    // MOV R4 R5
    // Type: Format I
    // Addressing Mode: Register Register
    // op: 0100 MOV
    // src: 0100 R4
    // ad: 0
    // b/w: 0
    // as: 00
    // dst: 0101 R5
    // cycles: 1
    // length: 1
    writeMemory16(0, 0b0100010000000101);
    // MOV R4 &ADDR
    // Type: Format I
    // Addressing Mode: Register Absolute
    // op: 0100 MOV
    // src: 0100 R4
    // ad: 1
    // b/w: 0
    // as: 00
    // dst: 0010 SR_REGNUM
    // cycles: 4 - 1
    // length: 2
    writeMemory16(1, 0b0100010010000010);
    writeMemory16(2, 0b0000000000000000);  // for absolute addressing

    // TEST -- MOV R4 R5
    spdlog::info("Testing MOV R4 R5");
    wait(1 * test.m_dut.m_cycleTime);
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 2);
    spdlog::info("PASSED");
    // TEST -- MOV R4 &ADDR
    spdlog::info("Testing MOV R4 &ADDR");
    wait(3 * test.m_dut.m_cycleTime);
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 6);
    spdlog::info("PASSED");

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
