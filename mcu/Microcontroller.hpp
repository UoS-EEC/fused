/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stdint.h>
#include <systemc>
#include "ps/PowerModelChannelIf.hpp"

/**
 * Interface/abstract class for microcontrollers/SoCs.
 * Mainly declares the methods and signals that the GDB server can use to
 * control and interrogate the MCU
 */
class Microcontroller : public sc_core::sc_module {
 public:
  /* ------ Ports ------ */
  sc_core::sc_in<bool> nReset{"nReset"};
  sc_core::sc_in<double> vcc{"vcc"};  // Supply voltage
  sc_core::sc_out<double> staticPower{"staticPower"};
  PowerModelEventOutPort powerModelPort{"powerModelPort"};

  /* ------ Signals ------ */

  /*------ Interrupt lines ------*/

  /* ------ Clocks ------ */
  /* ------ "Analog" signals ------ */
  /* ------ Miscellaneous ------ */

  //! Constructor
  explicit Microcontroller(sc_core::sc_module_name nm);

  /* ------ CPU control functions ------ */

  /**
   * @brief Stall CPU execution
   */
  virtual void stall(void) = 0;

  /**
   * @brief Unstall CPU execution
   */
  virtual void unstall(void) = 0;

  /**
   * @brief Check if CPU is stalled
   * @param true if CPU stalled, false otherwise
   */

  virtual bool isStalled(void) = 0;
  /**
   * @brief Execute a single instrucion, then stall
   */
  virtual void step(void) = 0;

  /**
   * @brief Reset target
   **/
  virtual void reset() = 0;

  /* ------ Interrogation functions ------ */
  /**
   * @brief dbgReadReg read the value of a CPU register
   * @param addr register number
   * @return value of register
   */
  virtual uint32_t dbgReadReg(size_t addr) = 0;

  /**
   * @brief dbgWriteReg write a value to a CPU register
   * @param addr register number
   * @param val write value
   */
  virtual void dbgWriteReg(size_t addr, uint32_t val) = 0;

  /**
   * @brief dbgReadMem Read device memory into buffer.
   * @param out Buffer to hold output.
   * @param addr Device address to read from.
   * @param len Number of bytes to read.
   * @return True if successful, false otherwise.
   */
  virtual bool dbgReadMem(uint8_t *out, size_t addr, size_t len) = 0;

  /**
   * @brief dbgWriteMem Write to device memory.
   * @param src Buffer holding bytes to write.
   * @param addr Device address to write to.
   * @param len Number of bytes to write.
   * @return True if successful, false otherwise.
   */
  virtual bool dbgWriteMem(uint8_t *src, size_t addr, size_t len) = 0;

  /* ------ Breakpoint functions ------ */

  /**
   * @brief insertBreakpoint insert a breakpoint at address addr.
   * Execution will halt when an instruction at this address is executed.
   * @param addr breakpoint address
   */
  virtual void insertBreakpoint(unsigned addr) = 0;

  /**
   * @brief removeBreakpoint insert a watchpoint at address addr.
   * Execution will halt on write to this address.
   * @param addr
   */
  virtual void removeBreakpoint(unsigned addr) = 0;

  /**
   * @brief pc_regnum get the PC register number
   */
  virtual uint32_t pc_regnum() = 0;

  /**
   * @brief pc_regnum get number of general purpose registers
   */
  virtual uint32_t n_regs() = 0;

  /**
   * @brief stop simulation
   */
  virtual void kill() { sc_core::sc_stop(); };
};
