/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "mcu/BankedMemory.hpp"
#include "mcu/Bus.hpp"
#include "mcu/BusTarget.hpp"
#include "mcu/Cache.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/DummyPeripheral.hpp"
#include "mcu/GenericMemory.hpp"
#include "mcu/Microcontroller.hpp"
#include "mcu/NonvolatileMemory.hpp"
#include "mcu/PowerController.hpp"
#include "mcu/VolatileMemory.hpp"
#include "mcu/cortex-m0/CortexM0Cpu.hpp"
#include "mcu/cortex-m0/Dma.hpp"
#include "mcu/cortex-m0/Gpio.hpp"
#include "mcu/cortex-m0/Nvic.hpp"
#include "mcu/cortex-m0/Spi.hpp"
#include "mcu/cortex-m0/SysTick.hpp"
#include "mcu/cortex-m0/UndoLogger.hpp"
#include "utilities/WriteTracker.hpp"
#include <array>
#include <iostream>
#include <ostream>
#include <stdint.h>
#include <systemc>
#include <utilities/SimpleMonitor.hpp>
#include <vector>

class MemicMicrocontroller : public Microcontroller {
  SC_HAS_PROCESS(MemicMicrocontroller);

public:
  /* ------ Signals ------ */

  /*------ Interrupt lines ------*/
  sc_core::sc_signal<int> cpu_active_exception{"cpu_active_exception"};
  sc_core::sc_signal<int> cpu_returning_exception{"cpu_returning_exception"};

  std::array<sc_core::sc_signal<bool>, 32> nvic_irq;
  sc_core::sc_signal<int> nvic_pending{"nvic_pending"};
  sc_core::sc_signal<bool> systick_irq{"systick_irq"};

  /* ------ Clocks ------ */
  ClockSourceChannel masterClock{
      "masterClock", sc_core::sc_time(0.8, sc_core::SC_US)}; // 1.25 MHz
  ClockSourceChannel peripheralClock{
      "peripheralClock", sc_core::sc_time(0.8, sc_core::SC_US)}; // 1.25 MHz

  /* ------ "Analog" signals ------ */

  /* ------ Miscellaneous ------ */
  sc_core::sc_signal<unsigned int> nvmWaitStates{"nvmWaitStates", 1};
  std::array<sc_core::sc_signal<bool>, 32> nResetGated;

  //! Signal from DMA to take over bus
  sc_core::sc_signal<bool> busStall{"busStall"};

  //! DMA triggers
  std::array<sc_core::sc_signal<bool>, 30> dmaTrigger;

  //! Constructor
  explicit MemicMicrocontroller(sc_core::sc_module_name nm);

  //! Destructor
  ~MemicMicrocontroller();

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

  /*
   * @brief debug print
   */
  friend std::ostream &operator<<(std::ostream &os,
                                  const MemicMicrocontroller &rhs);

private:
  /**
   * @brief process Monitors output from PMM and resets processor during
   * power outages
   */
  void process(void);

public:
  /* ------ Constants / internal pin mapping ------*/

  struct PowerControllerOutputAssignment {
    static const unsigned int Gpio0 = (1u << 0);
    static const unsigned int Gpio1 = (1u << 1);
    static const unsigned int Dma = (1u << 2);
    static const unsigned int Mon = (1u << 3);
    static const unsigned int Nvic = (1u << 4);
    static const unsigned int Scb = (1u << 5);
    static const unsigned int Spi = (1u << 6);
    static const unsigned int iSram = (1u << 7);
    static const unsigned int iSramCtrl = (1u << 8);
    static const unsigned int dSram = (1u << 9);
    static const unsigned int dSramCtrl = (1u << 10);
    static const unsigned int sysTick = (1u << 11);
  };

  /* ------ Peripherals ------ */
  DummyPeripheral *scb;              //! System Control Block is not implemented
  BankedMemory::BankedMemory *dsram; //! Volatile memory (SRAM)
  BankedMemory::BankedMemory *isram; //! Volatile memory (SRAM)
  BankedMemory::BankedMemory *invm;  //! Instruction memory (NVM)
  BankedMemory::BankedMemory *dnvm;  //! Data memory (NVRAM)
  Cache *icache;                     // [[maybe_unused]]
  Cache *dcache;                     // [[maybe_unused]]
  SysTick *sysTick;                  //! SysTick Timer
  Nvic *nvic;                        //! NVIC interrupt controller
  Spi *spi;                          //! SPI peripheral
  Gpio *gpio0;                       //! Basic GPIO
  Gpio *gpio1;                       //! Basic GPIO
  PowerController *powerController;
  UndoLogger *undoLogger;
  CortexM0Peripherals::Dma *dma; //! Direct Memory Access peripheral
  WriteTracker *writeTracker;    //! Tracks writes to  the attached memory

  /* ------- CPU & bus ------ */
  CortexM0Cpu m_cpu;
  Bus bus;
  std::vector<BusTarget *> slaves;

  /* ------ Simulation utilities */
  SimpleMonitor *mon;
};
