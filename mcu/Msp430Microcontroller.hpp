/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <iostream>
#include <systemc>
#include <vector>
#include "mcu/Bus.hpp"
#include "mcu/BusTarget.hpp"
#include "mcu/Cache.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/DummyPeripheral.hpp"
#include "mcu/GenericMemory.hpp"
#include "mcu/Microcontroller.hpp"
#include "mcu/NonvolatileMemory.hpp"
#include "mcu/VolatileMemory.hpp"
#include "mcu/msp430fr5xx/Adc12.hpp"
#include "mcu/msp430fr5xx/ClockSystem.hpp"
#include "mcu/msp430fr5xx/DigitalIo.hpp"
#include "mcu/msp430fr5xx/Dma.hpp"
#include "mcu/msp430fr5xx/Frctl_a.hpp"
#include "mcu/msp430fr5xx/InterruptArbiter.hpp"
#include "mcu/msp430fr5xx/Mpy32.hpp"
#include "mcu/msp430fr5xx/Msp430Cpu.hpp"
#include "mcu/msp430fr5xx/PowerManagementModule.hpp"
#include "mcu/msp430fr5xx/TimerA.hpp"
#include "mcu/msp430fr5xx/eUSCI_B.hpp"
#include "ps/EventLog.hpp"
#include "utilities/SimpleMonitor.hpp"

class Msp430Microcontroller : public Microcontroller {
  SC_HAS_PROCESS(Msp430Microcontroller);

 public:
  /* ------ Ports ------ */
  sc_core::sc_in<bool> nReset{"nReset"};
  sc_core::sc_inout<bool> ioPortA[16];  // Digital IO port A, aka 1 and 2
  sc_core::sc_inout<bool> ioPortB[16];  // Digital IO port B, aka 3 and 4
  sc_core::sc_inout<bool> ioPortC[16];  // Digital IO port C, aka 5 and 6
  sc_core::sc_inout<bool> ioPortD[16];  // Digital IO port D, aka 7 and 8

  /* ------ Signals ------ */
  sc_core::sc_signal<bool> dma_dummy{"dma_dummy"};

  /*------ Interrupt lines ------*/
  sc_core::sc_signal<bool> cpu_irq{"cpu_irq"};
  sc_core::sc_signal<bool> cpu_ira{"cpu_ira"};
  sc_core::sc_signal<unsigned> cpu_irqIdx{"cpu_irqIdx"};

  sc_core::sc_signal<bool> dma_irq{"dma_irq"};
  sc_core::sc_signal<bool> dma_ira{"dma_ira"};

  sc_core::sc_signal<bool> tima_irq{"tima_irq"};
  sc_core::sc_signal<bool> tima_ira{"tima_ira"};

  sc_core::sc_signal<bool> pmm_irq{"pmm_irq"};
  sc_core::sc_signal<bool> pmm_ira{"pmm_ira"};

  sc_core::sc_signal<bool> adc_irq{"adc_irq"};
  sc_core::sc_signal<bool> m_iraConnected{"m_iraConnected"};

  sc_core::sc_signal<bool> port1_irq{"port1_irq"};
  sc_core::sc_signal<bool> port2_irq{"port2_irq"};
  sc_core::sc_signal<bool> port3_irq{"port3_irq"};
  sc_core::sc_signal<bool> port4_irq{"port4_irq"};
  sc_core::sc_signal<bool> port5_irq{"port5_irq"};
  sc_core::sc_signal<bool> port6_irq{"port6_irq"};
  sc_core::sc_signal<bool> port7_irq{"port7_irq"};
  sc_core::sc_signal<bool> port8_irq{"port8_irq"};

  sc_core::sc_signal<bool> euscib_irq{"euscib_irq"};
  sc_core::sc_signal<bool> euscib_ira{"euscib_ira"};

  /* ------ Clocks ------ */
  ClockSourceChannel mclk{"mclk"};
  ClockSourceChannel smclk{"smclk"};
  ClockSourceChannel aclk{"aclk"};
  ClockSourceChannel vloclk{"vloclk"};
  ClockSourceChannel modclk{"modclk"};

  /* ------ "Analog" signals ------ */
  sc_core::sc_signal<double> vref{"vref"};

  /* ------ Miscellaneous ------ */
  //! Number of wait states in fram memory
  sc_core::sc_signal<unsigned int> framWaitStates{"framWaitStates"};

  //! Signal from DMA to take over bus
  sc_core::sc_signal<bool> cpuStall{"cpuStall"};

  //! DMA triggers
  std::array<sc_core::sc_signal<bool>, 30> dmaTrigger;

  //! Constructor
  explicit Msp430Microcontroller(sc_core::sc_module_name nm);

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
  virtual uint32_t pc_regnum() { return m_cpu.pc_regnum(); }

  /**
   * @brief Get number of general purpose registers
   * @retval number of gprs
   */
  virtual uint32_t n_regs() { return m_cpu.n_regs(); }

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

  /**
   * @brief process Monitors output from PMM and resets processor during
   * power outages
   */
  void process(void);

  /* ------ Constants ------ */

  /* ------ Private variables ------ */

  /* ------ Peripherals ------ */
  Adc12 *adc;
  ClockSystem *cs;
  DigitalIo *portA;
  DigitalIo *portB;
  DigitalIo *portC;
  DigitalIo *portD;
  DummyPeripheral *refgen;
  DummyPeripheral *watchdog;
  DummyPeripheral *portJ;
  NonvolatileMemory *fram;
  Cache *cache;
  eUSCI_B *euscib;
  Frctl_a *fram_ctl;
  GenericMemory *vectors;
  InterruptArbiter<37> *interruptArbiter;
  Mpy32 *mpy32;
  PowerManagementModule *pmm;
  TimerA *tima;
  VolatileMemory *sram;
  Dma *dma;
  SimpleMonitor *mon;

  /* ------- CPU & bus ------ */
  Msp430Cpu m_cpu;
  Bus bus;
  std::vector<BusTarget *> slaves;
};
