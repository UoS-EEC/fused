/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <systemc>
#include <tlm>
#include <unordered_set>
#include "mcu/ClockSourceIf.hpp"
#include "ps/EventLog.hpp"
#include "ps/PowerModelEventChannelIf.hpp"
#include "utilities/Utilities.hpp"

class Msp430Cpu : public sc_core::sc_module, tlm::tlm_bw_transport_if<> {
 public:
  /*------ Ports ------*/
  tlm::tlm_initiator_socket<> iSocket{"iSocket"};  //! TLM initiator socket
  sc_core::sc_port<ClockSourceConsumerIf> mclk{"mclk"};
  sc_core::sc_in<bool> irq{"irq_in"};         //! Interrupt request line
  sc_core::sc_in<unsigned> irqIdx{"irqIdx"};  //! Irq source index
  sc_core::sc_in<bool> pwrOn{"pwrOn"};        //! "power-good" signal
  sc_core::sc_in<bool> busStall{"busStall"};  //! indicate busy bus
  sc_core::sc_out<bool> ira{"ira_out"};       //! irq accepted

  //! Output port for power model events
  PowerModelEventOutPort powerModelEventPort{"powerModelEventPort"};

  //! Decides whether to wait for peripheral's ack of IRA asserted or
  //! just wait one cycle
  sc_core::sc_in<bool> iraConnected{"iraConnected"};

  //*------ Types ------*/
  typedef enum { WORD, BYTE } access_t;

  /*------ Methods ------*/
  SC_HAS_PROCESS(Msp430Cpu);

  Msp430Cpu(const sc_core::sc_module_name name, const bool logOperation = false,
            const bool logInstruction = false);

  /**
   * @brief end_of_elaboration SystemC callback. Used here for registering power
   * modelling events.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief writeMem: Callback function for write operations to memory by
   * emulator
   * @param val
   */
  void writeMem(const uint32_t addr, uint8_t *const data, const size_t bytelen);

  /**
   * @brief readMem: Callback function for read operations from memory by
   * emulator
   */
  void readMem(const uint32_t addr, uint8_t *const data, const size_t bytelen);

  /**
   * @brief waitCycles wait nCycles clock cycles.
   * @param nCycles  number of clock cycles to wait
   */
  void waitCycles(unsigned nCycles) {
    sc_core::wait(nCycles * mclk->getPeriod());
  }

  /**
   * @brief dbg_readReg Read register value without consuming simulation
   *  time.
   * @param addr Specify which register to read (0-15).
   * @return Value held in register.
   */
  uint16_t dbg_readReg(unsigned addr);

  /**
   * @brief dbg_writeReg Write to register without consuming simulation time.
   * @param val Value to write to register.
   * @param addr Specify which register to write to (0-15)
   */
  void dbg_writeReg(uint16_t addr, uint16_t val);

  /**
   * @brief stall Stall processor
   */
  void stall(void);

  /**
   * @brief unstall Unstall processor
   */
  void unstall(void);

  /**
   * @brief isStalled
   * @return True if processors is stalled, false otherwise.
   */
  bool isStalled(void);

  /**
   * @brief insertBreakPoint Insert a breakpoint at the specified address.
   * @param addr Address of breakpoint to be inserted.
   */
  void insertBreakpoint(unsigned addr);

  /**
   * @brief removeBreakpoint Remove a breakpoint at specified address.
   * @param addr Address of breakpoint to be removed.
   */
  void removeBreakpoint(unsigned addr);

  /**
   * @brief step Activate execution of a single instruction.
   */
  void step(void);
  /**
   * @brief reset Reset state to power-up default
   */
  void reset(void);

  /**
   * @brief Get PC register number
   * @retval PC register number
   */
  uint32_t pc_regnum() const { return PC_REGNUM; }

  /**
   * @brief Get number of general purpose registers
   * @retval number of gprs
   */
  uint32_t n_regs() const { return N_GPR; }

  /**
   * @brief operator<< state printout
   */
  friend std::ostream &operator<<(std::ostream &os, const Msp430Cpu &rhs);

  /*------ Dummy methods --------------------------------------------------*/

  // Dummy method:
  [[noreturn]] void invalidate_direct_mem_ptr(
      sc_dt::uint64 start_range[[maybe_unused]],
      sc_dt::uint64 end_range[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "invalidate_direct_mem_ptr not implemented");
    exit(1);
  }

      // Dummy method:
      [[noreturn]] tlm::tlm_sync_enum
      nb_transport_bw(tlm::tlm_generic_payload &trans[[maybe_unused]],
                      tlm::tlm_phase &phase[[maybe_unused]],
                      sc_core::sc_time &delay[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "nb_transport_bw is not implemented");
    exit(1);
  }
  /* ------ Public variables ------ */
 private:
  /* ------ Types ----- */

  typedef struct {
    size_t addr;   // Address/register idx
    uint32_t val;  // Value
    bool inMem;    // True if operand in memory, false if operand is register
    bool byteNotWord;  // True if byte-access, false if word access
  } operand_t;

  /* ------ Constants ------ */
  const bool m_doLogInstructions;
  const bool m_doLogOperation;

  static const unsigned PC_REGNUM = 0;
  static const unsigned SP_REGNUM = 1;
  static const unsigned SR_REGNUM = 2;
  static const unsigned CG_REGNUM = 3;
  static const unsigned N_GPR = 16;  // How many general purpose registers

  static const uint16_t OP_MOV = 0x4000;  // Opcode for MOV instruction

  /* ------ Private variables ------ */
  bool m_run{false};         //! Signal whether processor should run
  bool m_sleeping{false};    //! Indicate whether cpu is sleeping
  bool m_doStep{false};      //! Set to 1 to single-step, cleared automatically.
  uint64_t m_idleCycles{0};  //! Total number of idle cycles (for logging)
  EventLog &m_elog;
  EventLog::eventId m_idleCyclesEvent;
  EventLog::eventId m_formatIEvent;
  EventLog::eventId m_formatIIEvent;
  EventLog::eventId m_formatIIIEvent;
  EventLog::eventId m_pcIsDestinationEvent;  // Blanket for all branches/jumps
  EventLog::eventId m_irqEvent;
  std::map<std::string, EventLog::eventId> instrEventIds;

  std::array<uint32_t, 16> m_cpuRegs;

  std::unordered_set<unsigned> m_breakpoints;  //! Set of breakpoint addresses

  std::ofstream m_opsLogFile;    //! Log file (logs CPU op mode)
  std::ofstream m_instrLogFile;  //! Log file (executed instructions)

  /* ------ Private methods ------ */

  /**
   * @brief process SystemC thread -- main processing loop
   */
  [[noreturn]] void process(void);

  /**
   * @brief Msp430Cpu::processInterrupt Respond to an interrupt request and
   * call the corresponding interrupt routine.
   */
  void processInterrupt(void);

  /**
   * @brief handleBreakpoints Check if current PC matches any breakpoint.
   * @param pc
   * @return True if break point hit, otherwise false.
   */
  bool handleBreakpoints(unsigned pc);

  /**
   * @brief executeSingleOpInstruction execute format 1 instruction.
   * @param opcode
   */
  void executeSingleOpInstruction(uint16_t opcode);

  /**
   * @brief executeDoubleOpInstruction execute format 2 instruction.
   * @param opcode
   */
  void executeDoubleOpInstruction(uint16_t opcode);

  /**
   * @brief executeConditionalJump execute format 3 instruction.
   * @param opcode
   */
  void executeConditionalJump(uint16_t opcode);

  /**
   * @brief read16 read 16-bit value from bus
   * @return value read from memory, in host endianness.
   */
  uint16_t read16(size_t addr);

  /**
   * @brief read8 read 8-bit value from bus
   * @return value read from memory, in host endianness.
   */
  uint8_t read8(size_t addr);

  /**
   * @brief write16 write 16-bit value to bus
   * @param val value to write (in host endianness)
   */
  void write16(size_t addr, uint16_t val);

  /**
   * @brief write8 write 8-bit value to bus
   * @param val value to write (in host endianness)
   */
  void write8(size_t addr, uint8_t val);

  /**
   * @brief setGpr set value of cpu register.
   * @param addr address (register index)
   * @param val
   */
  void setGpr(size_t addr, uint32_t val) {
    assert(addr < m_cpuRegs.size());
    m_cpuRegs[addr] = val;
  }

  /**
   * @brief getGpr get value of cpu register.
   * @return value of cpu register
   */
  uint32_t getGpr(size_t addr) {
    assert(addr < m_cpuRegs.size());
    return m_cpuRegs[addr];
  }

  /**
   * @brief writeback perform writeback to register file or memory.
   * @param addr address/register index to write to
   * @param val  value to write
   * @param toMemory write to bus if true, write to register file if false
   * @param byteNotWord write  back single byte if false, otherwise full word.
   */
  void writeback(size_t addr, uint16_t val, bool toMemory, bool byteNotWord);

  /**
   * @brief writeback write back an operand to the register file/memory
   * @param operand
   */
  void writeback(operand_t operand);

  /**
   * @brief Load value of an operand from the register file/memory
   * @param operand
   */
  void loadOperand(operand_t &operand);

  /**
   * @brief getDestinationOperand get destination operand from bus/register
   * file
   * @param opcode
   */
  operand_t getDestinationOperand(uint16_t opcode);

  /**
   * @brief getSourceOperand get source operand from bus/register file
   * @param opcode
   * @retval operand
   */
  operand_t getSourceOperand(uint16_t opcode);

  /**
   * @brief check if source operand is a constant.
   * @param as source addressing mode
   * @param regIdx register index for source
   * @retval true if the source operand is a constant, false otherwise.
   */
  bool isSourceConstant(uint8_t as, uint8_t regIdx);

  /**
   * @brief getSoureceConstant get source constant value
   * @param as source addressing mode
   * @param regIdx source register index
   * @retval source constant value (16-bit)
   */
  uint16_t getSourceConstant(uint8_t as, uint8_t regIdx);

  /**
   * @brief isNegative check if a value is negative.
   * @param val value to check
   * @param byteNotWord value is a byte (true) or a word (false)
   * @retval true if the value is negative, false otherwise.
   */
  bool isNegative(uint16_t val, bool byteNotWord);

  /**
   * @brief isNegative check if a value is zero.
   * @param val value to check
   * @param byteNotWord value is a byte (true) or a word (false)
   * @retval true if the value is negative, false otherwise.
   */
  bool isZero(uint16_t val, bool byteNotWord);

  /**
   * @brief Check whether the addition a + b + c carries over
   * @param a
   * @param b
   * @param c carry bit
   * @param byteNotWord true if a and b are bytes, false if they are words.
   * @retval true if the addition a+b+c results in a carry, false otherwise.
   */
  bool isCarry(uint32_t a, uint32_t b, bool c, bool byteNotWord);

  /**
   * @brief Check whether the signed addition a + b + c overflows (i.e. sign of
   * result is wrong.)
   * @param a
   * @param b
   * @param c carry bit
   * @param byteNotWord true if a and b are bytes, false if they are words.
   * @retval true if the singed addition a+b+c overflows.
   */
  bool isOverflow(uint32_t a, uint32_t b, bool c, bool byteNotWord);

  /**
   * @brief fetch Load a word from memory at PC, and increment PC
   * @retval value fetched from memory
   */
  uint16_t fetch();

  /**
   * @brief getPc get program counter
   * @retval program counter
   */
  uint32_t getPc() { return m_cpuRegs[PC_REGNUM]; }

  /**
   * @brief setPc set program counter
   * @param val new program counter value
   */
  void setPc(uint32_t val) {
    assert(val % 2 == 0);
    m_cpuRegs[PC_REGNUM] = val;
  }

  /**
   * @brief getSr get status register
   * @retval program counter
   */
  uint32_t getSr() { return m_cpuRegs[SR_REGNUM]; }

  /**
   * @brief setSr set status register
   * @param val new status register value
   */
  void setSr(uint32_t val) { m_cpuRegs[SR_REGNUM] = val; }

  /**
   * @brief getSp get stack pointer
   * @retval stack pointer
   */
  uint32_t getSp() { return m_cpuRegs[SP_REGNUM]; }

  /**
   * @brief setSp set stack pointer
   * @param val new stack pointer value
   */
  void setSp(uint32_t val) { m_cpuRegs[SP_REGNUM] = val; }
  /**
   * @brief getOverflowFlag get overflow flag (V)
   * @retval overflow flag
   */
  bool getOverflowFlag() { return m_cpuRegs[SR_REGNUM] & (1u << 8); }

  /**
   * @brief setOverflowFlag set overflow flag (V)
   * @param overflow flag
   */
  void setOverflowFlag(bool val) {
    m_cpuRegs[SR_REGNUM] = Utility::setBit(8, m_cpuRegs[SR_REGNUM], val);
  }

  /**
   * @brief getNegativeFlag get negative flag (N)
   * @retval negative flag
   */
  bool getNegativeFlag() { return m_cpuRegs[SR_REGNUM] & (1u << 2); }

  /**
   * @brief setNegativeFlag set negative flag (N)
   * @param negative flag
   */
  void setNegativeFlag(bool val) {
    m_cpuRegs[SR_REGNUM] = Utility::setBit(2, m_cpuRegs[SR_REGNUM], val);
  }

  /**
   * @brief getZeroFlag get zero flag (Z)
   * @retval zero flag
   */
  bool getZeroFlag() { return m_cpuRegs[SR_REGNUM] & (1u << 1); }

  /**
   * @brief setZeroFlag set zero flag (Z)
   * @param negative flag
   */
  void setZeroFlag(bool val) {
    m_cpuRegs[SR_REGNUM] = Utility::setBit(1, m_cpuRegs[SR_REGNUM], val);
  }

  /**
   * @brief getCarryFlag get zero flag (C)
   * @retval zero flag
   */
  bool getCarryFlag() { return m_cpuRegs[SR_REGNUM] & (1u << 0); }

  /**
   * @brief setCarryFlag set zero flag (Z)
   * @param negative flag
   */
  void setCarryFlag(bool val) {
    m_cpuRegs[SR_REGNUM] = Utility::setBit(0, m_cpuRegs[SR_REGNUM], val);
  }

  /**
   * @brief waitForCommand Stall simulation while waiting for command (e.g. from
   * debugger).
   */
  void waitForCommand();
};
