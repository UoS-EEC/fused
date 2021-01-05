/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <deque>
#include <systemc>
#include <tlm>
#include <unordered_set>
#include "mcu/ClockSourceIf.hpp"
#include "ps/PowerModelChannelIf.hpp"

extern "C" {
#include "mcu/cortex-m0/decode.h"
#include "mcu/cortex-m0/exmemwb.h"
}

extern struct CPU cpu;

class CortexM0Cpu : public sc_core::sc_module, tlm::tlm_bw_transport_if<> {
  SC_HAS_PROCESS(CortexM0Cpu);

 public:
  /* ------ Ports ------ */
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};  //! CPU clock
  tlm::tlm_initiator_socket<> iSocket;                 //! TLM initiator socket
  sc_core::sc_in<bool> pwrOn{"pwrOn"};                 //! "power-good" signal

  //! Output port for power model events
  PowerModelEventOutPort powerModelPort{"powerModelPort"};

  // -- Interrupts
  // SysTick
  sc_core::sc_in<bool> sysTickIrq{"sysTickIrq"};

  // NVIC
  sc_core::sc_in<int> nvicIrq{"nvicIrq"};

  // Output/feedback
  sc_core::sc_out<int> returningException{"returningException"};
  sc_core::sc_out<int> activeException{"activeException"};

  /* ------ Types ------ */

  /* ------ Methods ------ */
  /**
   * @brief CortexM0Cpu constructor
   */
  CortexM0Cpu(const sc_core::sc_module_name nm);

  /**
   * @brief end_of_elaboration SystemC callback. Used here for registering power
   * modelling events as well as setting up SC_THREADs and SC_METHODs.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief exceptionCheck Check for pending exceptions, and handle them.
   */
  void exceptionCheck();

  /**
   * @brief exceptionEnter Enter the handler corresponding to exceptionId.
   * @param  exceptionId ID of pending exception to be handled.
   */
  void exceptionEnter(const unsigned exceptionId);

  /**
   * @brief exceptionReturn Return from an exception.
   * @param EXC_RETURN value loaded into PC to signal an exception return.
   */
  void exceptionReturn(const uint32_t EXC_RETURN);

  /**
   * @brief read_cb adds CPP context to C callback
   * @param addr read address (MCU memory space)
   * @param data buffer for return value
   * @param bytelen number of bytes to be read
   */
  static void read_cb(const uint32_t addr, uint8_t *const data,
                      const size_t bytelen);

  /**
   * @brief write_cb adds CPP context to C callback
   * @param addr write address (MCU memory space)
   * @param data buffer for return value
   * @param bytelen number of bytes to be written
   */
  static void write_cb(const uint32_t addr, uint8_t *const data,
                       const size_t bytelen);

  /**
   * @brief consume_cycles_cb Call SC wait() to wait for n clock cycles
   * @param n number of cycles to consume
   */
  static void consume_cycles_cb(const size_t n);

  /**
   * @brief exception_return_cb Call adds C++ context to C callback of
   * exceptionReturn
   */
  static void exception_return_cb(const uint32_t EXC_RETURN);

  /**
   * @brief exception_return_cb Call adds C++ context to C callback of
   * getNextPipelineInst
   */
  static uint16_t next_pipeline_instr_cb();

  /**
   * @brief getNextPipelineInst Used for getting second half of 32-bit
   * instructions. Pops an instruction from the pipeline and inserts a NOP in
   * its place.
   * @retval next instruction from the pipeline
   */
  uint16_t getNextPipelineInstr();

  /**
   * @brief writeMem: Callback function for write operations to memory by
   * emulator
   * @param addr write address (MCU memory space)
   * @param data buffer for return value
   * @param bytelen number of bytes to be read
   */
  void writeMem(const uint32_t addr, uint8_t *const data, const size_t bytelen);

  /**
   * @brief write32 Utility function to write a word to memory.
   * @param addr address to write to
   * @param val value to write
   */
  void write32(const uint32_t addr, const uint32_t val);

  /**
   * @brief read32 Utility function to read a word from memory.
   * @param addr address to read from
   * @retval Value read from memory (bus)
   */
  uint32_t read32(const uint32_t addr);

  /**
   * @brief readMem: Callback function for read operations from memory by
   * emulator
   * @param addr read address (MCU memory space)
   * @param data buffer for return value
   * @param bytelen number of bytes to be read
   */
  void readMem(const uint32_t addr, uint8_t *const data, const size_t bytelen);

  /* ------ Controls for GDB server ------ */

  /**
   * @brief dbg_readReg Read register value without consuming simulation
   *  time.
   * @param addr Specify which register to read (0-15).
   * @return Value held in register.
   */
  uint32_t dbg_readReg(size_t addr);

  /**
   * @brief dbg_writeReg Write to register without consuming simulation
   * time.
   * @param data Value to write to register.
   * @param addr Specify which register to write to (0-15)
   */
  void dbg_writeReg(size_t addr, uint32_t data);

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
   * @brief operator<< debug printout
   */
  friend std::ostream &operator<<(std::ostream &os, const CortexM0Cpu &rhs);

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

  /* ------ Public variables ------*/
 private:
  /* ------ Constants ------ */
  static const unsigned N_GPR = 16;      // How many general purpose registers
  static const unsigned SP_REGNUM = 13;  // Stack pointer
  static const unsigned LR_REGNUM = 14;  // Link register
  static const unsigned PC_REGNUM = 15;  // Program counter
  static const unsigned CPSR_REGNUM = 0x19;  // (virtual register)

  static const unsigned OPCODE_WFE = 0xbf20;  //! Wait for Event opcode
  static const unsigned OPCODE_WFI = 0xbf30;  //! Wait for Interrupt opcode
  static const unsigned OPCODE_NOP = 0x46c0;  // 0xbf00;  //! NOP (mov r8, r8)

  /* ------ Private variables ------ */
  struct InstructionBuffer {
    unsigned data{0};
    unsigned address{0};
    bool valid{false};
  };

  std::deque<uint16_t> m_instructionQueue{};  //! Pipeline
  int m_bubbles{0};  //! Current number of pipeline bubbles
  int m_pipelineStages;
  bool m_sleeping{false};
  bool m_run{false};
  bool m_doStep{false};
  InstructionBuffer m_instructionBuffer;
  std::unordered_set<unsigned> m_breakpoints;  // Set of breakpoint addresses
  std::unordered_set<unsigned> m_watchpoints;  // Set of watchpoint addresses
  std::array<unsigned, 17> m_regsAtExceptEnter{{0}};  //! Used for checking

  /* Power model event & state ids */
  int m_idleCyclesEventId;     //! Event used to track idle cycles
  int m_nInstructionsEventId;  //! Event used to track number of
                               //! executed instructions
  int m_offStateId;
  int m_onStateId;
  int m_sleepStateId;

  /* ------ Private methods ------ */

  /**
   * @brief process SystemC thread -- main processing loop
   */
  [[noreturn]] void process();

  /**
   * @brief reset Reset state to power-up default
   */
  void reset();

  /**
   * @brief powerOffChecks Perform checks when power is lost, issue warning if
   * CPU is active.
   */
  void powerOffChecks();

  /**
   * @brief wait for a command from controller.
   */
  void waitForCommand();

  /**
   * @brief flushPipeline flush the instruction queue and insert nops
   */
  void flushPipeline();

  /**
   *@brief fetch fetch an instruction via the instructino buffer.
   *@param address memory address of instruction
   *@retval instruction at address.
   */
  unsigned fetch(const unsigned address);

  /**
   * @brief getNextExecutionPc get the address of the next instruction to be
   * executed. This value is PC adjusted for pipeline and bubbles.
   * @retval address of next instruction to be executed.
   */
  unsigned getNextExecutionPc() const;
};
