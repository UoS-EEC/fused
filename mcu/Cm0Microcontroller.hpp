/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <array>
#include <iostream>
#include <systemc>
#include <utilities/SimpleMonitor.hpp>
#include <vector>
#include "include/fused.h"
#include "mcu/Bus.hpp"
#include "mcu/BusTarget.hpp"
#include "mcu/Cache.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/DummyPeripheral.hpp"
#include "mcu/GenericMemory.hpp"
#include "mcu/Microcontroller.hpp"
#include "mcu/NonvolatileMemory.hpp"
#include "mcu/VolatileMemory.hpp"
#include "mcu/cortex-m0/CortexM0Cpu.hpp"
#include "mcu/cortex-m0/Nvic.hpp"
#include "mcu/cortex-m0/OutputPort.hpp"
#include "mcu/cortex-m0/Spi.hpp"
#include "mcu/cortex-m0/SysTick.hpp"

class Cm0Microcontroller : public Microcontroller {
  SC_HAS_PROCESS(Cm0Microcontroller);

 public:
  /* ------ Ports ------ */
  sc_core::sc_in<bool> nReset{"nReset"};
  std::array<sc_core::sc_in<bool>, 16> externalIrq;

  /* ------ Signals ------ */

  /*------ Interrupt lines ------*/
  sc_core::sc_signal<int> cpu_active_exception{"cpu_active_exception"};
  sc_core::sc_signal<int> cpu_returning_exception{"cpu_returning_exception"};

  std::array<sc_core::sc_signal<bool>, 16> nvic_irq;
  sc_core::sc_signal<int> nvic_pending{"nvic_pending"};
  sc_core::sc_signal<bool> systick_irq{"systick_irq"};

  /* ------ Clocks ------ */
  ClockSourceChannel masterClock{
      "masterClock", sc_core::sc_time(125, sc_core::SC_NS)};  // 8 MHz
  ClockSourceChannel peripheralClock{
      "peripheralClock", sc_core::sc_time(125, sc_core::SC_NS)};  // 8 MHz

  /* ------ "Analog" signals ------ */

  /* ------ Miscellaneous ------ */
  sc_core::sc_signal<unsigned int> nvmWaitStates{"nvmWaitStates", 1};

  //! Constructor
  explicit Cm0Microcontroller(sc_core::sc_module_name nm);

  /* ------ CPU control functions ------ */

  /**
   * @brief reset target.
   */
  virtual void reset() {
    spdlog::error("MCU: reset() Not implemented.");
    sc_core::sc_stop();
  }

  /**
   * @brief Stall CPU execution
   */
  virtual void stall(void) override { m_cpu.stall(); }

  /**
   * @brief Unstall CPU execution
   */
  virtual void unstall(void) override { m_cpu.unstall(); }

  /**
   * @brief Check if CPU is stalled
   * @param true if CPU stalled, false otherwise
   */

  virtual bool isStalled(void) override { return m_cpu.isStalled(); }

  /**
   * @brief Execute a single instrucion, then stall
   */
  virtual void step(void) override { m_cpu.step(); }

  /* ------ Interrogation functions ------ */

  /**
   * @brief Get PC register number
   * @retval PC register number
   */
  virtual uint32_t pc_regnum() override { return m_cpu.pc_regnum(); }

  /**
   * @brief Get number of general purpose registers
   * @retval number of gprs
   */
  virtual uint32_t n_regs() override { return m_cpu.n_regs(); }

  /**
   * @brief dbgReadReg read the value of a CPU register
   * @param addr register number
   * @return value of register
   */
  virtual uint32_t dbgReadReg(size_t addr) override {
    return m_cpu.dbg_readReg(addr);
  }

  /**
   * @brief dbgWriteReg write a value to a CPU register
   * @param addr register number
   * @param val write value
   */
  virtual void dbgWriteReg(size_t addr, uint32_t val) override {
    m_cpu.dbg_writeReg(addr, val);
  }

  /**
   * @brief dbgReadMem Read device memory into buffer.
   * @param out Buffer to hold output.
   * @param addr Device address to read from.
   * @param len Number of bytes to read.
   * @return True if successful, false otherwise.
   */
  virtual bool dbgReadMem(uint8_t *out, size_t addr, size_t len) override;

  /**
   * @brief dbgWriteMem Write to device memory.
   * @param src Buffer holding bytes to write.
   * @param addr Device address to write to.
   * @param len Number of bytes to write.
   * @return True if successful, false otherwise.
   */
  virtual bool dbgWriteMem(uint8_t *src, size_t addr, size_t len) override;

  /* ------ Breakpoint functions ------ */

  /**
   * @brief insertBreakpoint insert a breakpoint at address addr.
   * Execution will halt when an instruction at this address is executed.
   * @param addr breakpoint address
   */
  virtual void insertBreakpoint(unsigned addr) override {
    m_cpu.insertBreakpoint(addr);
  }

  /**
   * @brief removeBreakpoint insert a watchpoint at address addr.
   * Execution will halt on write to this address.
   * @param addr
   */
  virtual void removeBreakpoint(unsigned addr) override {
    m_cpu.removeBreakpoint(addr);
  }

 private:
  /**
   * @brief process Monitors output from PMM and resets processor during
   * power outages
   */
  void process(void);

 public:
  /* ------ Constants ------ */

  /* ------ Peripherals ------ */
  DummyPeripheral *scb;     //! System Control Block is not implemented
  VolatileMemory *sram;     //! Volatile memory (SRAM)
  NonvolatileMemory *invm;  //! Instruction memory (NVM)
  NonvolatileMemory *dnvm;  //! Data memory (NVRAM)
  SysTick *sysTick;         //! SysTick Timer
  Nvic *nvic;               //! NVIC interrupt controller
  OutputPort *outputPort;   //! Basic output "IO" port
  Spi *spi;                 //! SPI peripheral

  /* ------- CPU & bus ------ */
  CortexM0Cpu m_cpu;
  Bus bus;
  std::vector<BusTarget *> slaves;

  /* ------ Simulation utilities */
  SimpleMonitor *mon;
};
