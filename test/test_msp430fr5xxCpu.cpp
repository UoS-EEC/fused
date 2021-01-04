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
#include "mcu/msp430fr5xx/Msp430Cpu.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "ps/PowerModelChannel.hpp"
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
  GenericMemory mem{"mem", 0, 0xFFFF};  //! 65k memory
  ClockSourceChannel mclk{"mclk", sc_time(125, SC_NS)};
  PowerModelChannel powerModelChannel{
      "powerModelChannel", "/tmp/testPowerModelChannel.csv", sc_time(1, SC_US)};

  SC_CTOR(dut) {
    mem.pwrOn.bind(nreset);
    mem.tSocket.bind(m_dut.iSocket);
    mem.systemClk.bind(mclk);
    mem.powerModelEventPort.bind(powerModelChannel);
    m_dut.mclk.bind(mclk);
    m_dut.pwrOn.bind(nreset);
    m_dut.irq.bind(irq);
    m_dut.ira.bind(ira);
    m_dut.irqIdx.bind(irqIdx);
    m_dut.iraConnected.bind(iraConnected);
    m_dut.busStall.bind(stallCpu);
    m_dut.powerModelEventPort.bind(powerModelChannel);
  }

  void reset() {
    nreset.write(false);
    wait(5 * mclk.getPeriod());
    nreset.write(true);
    wait(SC_ZERO_TIME);
    m_dut.dbg_writeReg(SR_REGNUM, 0x00);  // Clear CPUOFF flag
  }

  // CPU constants
  static const unsigned PC_REGNUM = 0;
  static const unsigned SP_REGNUM = 1;
  static const unsigned SR_REGNUM = 2;
  static const unsigned CG_REGNUM = 3;
  static const unsigned N_GPR = 16;  // How many general purpose registers

  Msp430Cpu m_dut{"dut"};
};

SC_MODULE(tester) {
 public:
  SC_CTOR(tester) { SC_THREAD(runtests); }

  /*
   * Run instructions one by one & check results and timing. A NOP should be
   * inserted between each instruction.
   */
  void runtests() {
    test.m_dut.unstall();

    /**************************************************************************
     * Format I (Double-operand)
     *************************************************************************/

    // Register direct -> others
    // -------------------------

    // TEST -- MOV.B rs, rd
    spdlog::info("TEST: MOV.B r12, r13");
    test.reset();
    test.m_dut.dbg_writeReg(12, 0xabcd);  // Set source value
    writeMemory16(0, 0x4c4d);             // MOV.B r12, r13
    writeMemory16(2, 0x4303);             // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(1 * test.m_dut.mclk->getPeriod());  // Execute MOV
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(13) == 0xcd);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 2);

    // TEST -- MOV rs, #ofs(rd)
    spdlog::info("TEST: MOV r12, 4(r1)");
    test.reset();
    test.m_dut.dbg_writeReg(12, 0xabcd);  // Set source value
    test.m_dut.dbg_writeReg(1, 6);        // Set source value
    writeMemory16(0, 0x4c81);             // MOV r12, #ofs(r1)
    writeMemory16(2, 0x0004);             // #ofs = 4
    writeMemory16(4, 0x4303);             // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(3 * test.m_dut.mclk->getPeriod());  // Execute MOV
    wait(SC_ZERO_TIME);
    sc_assert(readMemory16(10) == 0xabcd);  // Check destination value
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 4);

    // TEST -- MOV rs, &abs

    spdlog::info("TEST: MOV r12, &0x000a");
    test.reset();
    test.m_dut.dbg_writeReg(12, 0xabcd);  // Set source value
    writeMemory16(0, 0x4cc2);             // MOV r12, &abs
    writeMemory16(2, 0x000a);             // &abs = 0x000a
    writeMemory16(4, 0x4303);             // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(3 * test.m_dut.mclk->getPeriod());  // Execute MOV
    wait(SC_ZERO_TIME);
    sc_assert(readMemory16(0x000a) == 0xabcd);  // Check destination value
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 4);

    // TEST -- BR #immediate (mov dst, PC)
    spdlog::info("TEST: BR #0x000a (MOV #0x000a, PC)");
    test.reset();
    writeMemory16(0, 0x4030);   // BR #imm
    writeMemory16(2, 0x000a);   // #imm=0x000a
    writeMemory16(10, 0x4303);  // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(3 * test.m_dut.mclk->getPeriod());  // Execute BR
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 0x000a);

    // Register indirect -> others (e.g. mov @r1, r2)
    // ----------------------------------------------

    // TEST -- xor.b @rs, #ofs(rd)
    spdlog::info("TEST: XOR.b @r13, 0(r12)");
    test.reset();
    test.m_dut.dbg_writeReg(13, 6);   // source address
    test.m_dut.dbg_writeReg(12, 10);  // destination address
    writeMemory16(0, 0xedec);         // xor.b @r13, #ofs(r12)
    writeMemory16(2, 0x0000);         // #ofs = 0
    writeMemory16(4, 0x4303);         // NOP
    writeMemory16(6, 0xabcd);         // source value
    writeMemory16(10, 0xdcba);        // destination value

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(4 * test.m_dut.mclk->getPeriod());  // Execute MOV.B
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 4);
    spdlog::info("XOR result: 0x{:04x}", readMemory16(10));
    sc_assert(readMemory16(10) == 0xdc77);  // Should this be 0x0077?

    // Indexed -> others
    // -----------------

    // TEST -- MOV.B ofs(rs), rd
    spdlog::info("TEST: MOV.B 10(r1), r12");
    test.reset();
    test.m_dut.dbg_writeReg(1, 0);  // source address
    writeMemory16(0, 0x415c);       // MOV.B #ofs(r1), r12
    writeMemory16(2, 0x000a);       // #ofs = 10
    writeMemory16(4, 0x4303);       // NOP
    writeMemory16(0xa, 0xabcd);     // source value

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(3 * test.m_dut.mclk->getPeriod());  // Execute MOV.B
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(12) == 0x00cd);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 4);

    // Immediate -> others
    // -----------------

    // TEST -- AND #imm, rn
    spdlog::info("TEST: AND #255, r12");
    test.reset();
    test.m_dut.dbg_writeReg(12, 0xAAAA);
    writeMemory16(0, 0xf03c);  // AND #imm, r12
    writeMemory16(2, 0x00ff);  // imm=255
    writeMemory16(4, 0x4303);  // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(2 * test.m_dut.mclk->getPeriod());  // Execute AND
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(12) == 0xaa);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 4);

    /**************************************************************************
     * Format II (Single-operand)
     *************************************************************************/
    // TEST -- CALL rn
    spdlog::info("TEST: CALL r8");
    test.reset();
    test.m_dut.dbg_writeReg(SP_REGNUM, 0xaa);  // Initialise SP
    test.m_dut.dbg_writeReg(8, 0xa);           // Call address
    writeMemory16(0, 0x1288);                  // CALL r8
    writeMemory16(0xa, 0x4303);                // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(4 * test.m_dut.mclk->getPeriod());  // Execute CALL
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 0xa);   // PC == Call target
    sc_assert(test.m_dut.dbg_readReg(SP_REGNUM) == 0xa8);  // SP-=2
    sc_assert(readMemory16(0xa8) == 0x02);  // Check return PC pushed

    // TEST -- PUSH rn
    spdlog::info("TEST: PUSH r8");
    test.reset();
    test.m_dut.dbg_writeReg(SP_REGNUM, 0xaa);  // Initialise SP
    test.m_dut.dbg_writeReg(8, 0xaabb);        // Initialise value in r8 to push
    writeMemory16(0, 0x1208);                  // Push r8
    writeMemory16(2, 0x4303);                  // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(3 * test.m_dut.mclk->getPeriod());  // Execute PUSH
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 0x2);
    sc_assert(test.m_dut.dbg_readReg(SP_REGNUM) == 0xa8);
    sc_assert(readMemory16(0xa8) == 0xaabb);

    // TEST -- CALL #immediate
    spdlog::info("TEST: CALL #N");
    test.reset();
    test.m_dut.dbg_writeReg(SP_REGNUM, 0xaa);  // Initialise SP
    writeMemory16(0, 0x12b0);                  // CALL #imm
    writeMemory16(2, 0x0010);                  // #imm=0x0010
    writeMemory16(0x10, 0x4303);               // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(4 * test.m_dut.mclk->getPeriod());  // Execute CALL
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 0x10);
    sc_assert(test.m_dut.dbg_readReg(SP_REGNUM) == 0xa8);
    sc_assert(readMemory16(0xa8) == 0x04);

    /**************************************************************************
     * Format III (Jump)
     *************************************************************************/

    // TEST -- jc $+26 (jz 26(PC)) -- taken
    spdlog::info("TEST: JZ $-20 (aka JZ 26(PC)) -- taken");
    test.reset();
    test.m_dut.dbg_writeReg(12, 0xAAAA);
    test.m_dut.dbg_writeReg(SR_REGNUM, Z);  // Set zero flag
    writeMemory16(0, 0x240c);               // JZ $+26
    writeMemory16(26, 0x4303);              // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(2 * test.m_dut.mclk->getPeriod());  // Execute JUMP
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 26);

    // TEST -- jc $+26 (jz 26(PC))  -- not taken
    spdlog::info("TEST: JZ $-20 (aka JZ 26(PC)) -- not taken");
    test.reset();
    test.m_dut.dbg_writeReg(12, 0xAAAA);
    // Don't set zero flag
    writeMemory16(0, 0x240c);  // JZ $+26
    writeMemory16(2, 0x4303);  // NOP

    wait(1 * test.m_dut.mclk->getPeriod());  // 1 Cycle of sleep after reset
    wait(2 * test.m_dut.mclk->getPeriod());  // Execute JUMP (not taken)
    wait(SC_ZERO_TIME);
    sc_assert(test.m_dut.dbg_readReg(PC_REGNUM) == 2);

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
