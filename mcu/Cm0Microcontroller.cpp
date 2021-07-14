/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "include/cm0-fused.h"
#include "mcu/Cm0Microcontroller.hpp"
#include "mcu/Microcontroller.hpp"
#include "mcu/cortex-m0/CortexM0Cpu.hpp"
#include "mcu/cortex-m0/Dma.hpp"
#include "mcu/cortex-m0/Gpio.hpp"
#include "mcu/cortex-m0/Nvic.hpp"
#include "mcu/cortex-m0/SysTick.hpp"
#include "utilities/Config.hpp"
#include <iomanip>
#include <iostream>
#include <numeric>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdint.h>
#include <string>
#include <systemc>
#include <tlm>

using namespace sc_core;
using namespace CortexM0Peripherals;

Cm0Microcontroller::Cm0Microcontroller(sc_module_name nm)
    : Microcontroller(nm), m_cpu("CPU", ROM_START), bus("bus"),
      masterClock("masterClock", sc_time::from_seconds(Config::get().getDouble(
                                     "MasterClockPeriod"))),
      peripheralClock("peripheralClock",
                      sc_time::from_seconds(
                          Config::get().getDouble("PeripheralClockPeriod"))) {
  /* ------ Memories ------ */
  invm = new NonvolatileMemory("invm", ROM_START, ROM_START + ROM_SIZE - 1);
  dnvm =
      new NonvolatileMemory("dnvm", NVRAM_START, NVRAM_START + NVRAM_SIZE - 1);

  sram = new VolatileMemory("sram", SRAM_START, SRAM_START + SRAM_SIZE - 1);

  /* ------ Peripherals ------ */
  scb = new DummyPeripheral("scb", 0xe000ed00, 0xe000ed8f);
  sysTick = new SysTick("sysTick");
  nvic = new Nvic("nvic");
  mon = new SimpleMonitor("mon", SIMPLE_MONITOR_BASE);
  gpio = new Gpio("gpio", GPIO_BASE, GPIO_EXCEPT_ID);
  spi = new Spi("spi", SPI1_BASE, SPI1_BASE + 0x10);
  dma = new Dma("dma", DMA_BASE);

  slaves.push_back(dnvm);
  slaves.push_back(dma);
  slaves.push_back(invm);
  slaves.push_back(gpio);
  slaves.push_back(mon);
  slaves.push_back(nvic);
  slaves.push_back(scb);
  slaves.push_back(spi);
  slaves.push_back(sram);
  slaves.push_back(sysTick);

  /* ------------------------ */

  m_cpu.clk.bind(masterClock);
  for (const auto &s : slaves) {
    s->systemClk.bind(masterClock);
  }

  /* ------ Bind ------ */

  // IO

  // Clocks
  sysTick->clk.bind(masterClock);
  spi->clk.bind(peripheralClock);
  gpio->clk.bind(peripheralClock);

  // Interrupts
  m_cpu.activeException.bind(cpu_active_exception);
  m_cpu.returningException.bind(cpu_returning_exception);
  m_cpu.sysTickIrq.bind(systick_irq);
  m_cpu.nvicIrq.bind(nvic_pending);

  sysTick->irq.bind(systick_irq);
  sysTick->returning_exception.bind(cpu_returning_exception);

  spi->irq.bind(nvic_irq[SPI1_EXCEPT_ID]);
  spi->active_exception.bind(cpu_active_exception);

  gpio->irq.bind(nvic_irq[GPIO_EXCEPT_ID]);
  gpio->active_exception.bind(cpu_active_exception);

  dma->irq.bind(nvic_irq[DMA_EXCEPT_ID]);
  dma->active_exception.bind(cpu_active_exception);

  nvic->pending.bind(nvic_pending);
  nvic->returning.bind(cpu_returning_exception);
  nvic->active.bind(cpu_active_exception);

  // Bind internal interrupt signals
  for (unsigned i = 0; i < nvic_irq.size(); i++) {
    nvic->irq[i].bind(nvic_irq[i]);
  }

  // Reset
  m_cpu.pwrOn.bind(nReset);
  for (int i = 0; i < slaves.size(); ++i) {
    slaves[i]->pwrOn.bind(nReset);
  }

  // Events for power model
  m_cpu.powerModelPort.bind(powerModelPort);
  for (const auto &s : slaves) {
    s->powerModelPort.bind(powerModelPort);
  }

  // Miscellaneous
  invm->waitStates.bind(nvmWaitStates);
  dnvm->waitStates.bind(nvmWaitStates);
  m_cpu.busStall.bind(busStall);

  // DMA
  dma->busStall.bind(busStall);
  for (size_t i = 0; i < dmaTrigger.size(); ++i) {
    dma->trigger[i].bind(dmaTrigger[i]);
  }

  // Bus
  m_cpu.iSocket.bind(bus.tSocket);
  dma->iSocket.bind(bus.tSocket);
  for (const auto &s : slaves) {
    bus.bindTarget(*s);
  }
}

Cm0Microcontroller::~Cm0Microcontroller() {
  delete scb;
  delete gpio;
  delete dma;
  delete mon;
  delete nvic;
  delete spi;
  delete sram;
  delete sysTick;
  delete invm;
  delete dnvm;
}

bool Cm0Microcontroller::dbgReadMem(uint8_t *out, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;

  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(out);
  trans.set_command(tlm::TLM_READ_COMMAND);
  return bus.transport_dbg(0, trans);
}

bool Cm0Microcontroller::dbgWriteMem(uint8_t *src, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;
  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(src);
  trans.set_command(tlm::TLM_WRITE_COMMAND);

  return bus.transport_dbg(0, trans);
}

std::ostream &operator<<(std::ostream &os, const Cm0Microcontroller &rhs) {
  os << "<Cm0Microcontroller> " << rhs.name() << "\n";
  os << "Memory map:\n"
     << "address(start)    address(end)  Name\n";
  for (unsigned int i = 0; i < rhs.slaves.size(); i++) {
    os << fmt::format("0x{:08x}        0x{:08x}    {:s}\n",
                      rhs.slaves[i]->startAddress(),
                      rhs.slaves[i]->endAddress(), rhs.slaves[i]->name());
  }
  return os;
}
