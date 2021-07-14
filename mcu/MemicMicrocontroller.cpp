/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "include/memic-fused.h"
#include "mcu/Cache.hpp"
#include "mcu/MemicMicrocontroller.hpp"
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

MemicMicrocontroller::MemicMicrocontroller(sc_module_name nm)
    : Microcontroller(nm), m_cpu("CPU", ROM_START), bus("bus"),
      masterClock("masterClock", sc_time::from_seconds(Config::get().getDouble(
                                     "MasterClockPeriod"))),
      peripheralClock("peripheralClock",
                      sc_time::from_seconds(
                          Config::get().getDouble("PeripheralClockPeriod"))) {
  /* ------ Memories ------ */
  invm = new BankedMemory::BankedMemory("invm", ROM_START, INVM_CTRL_BASE, 32,
                                        ROM_SIZE / 32, /*isVolatile=*/false);

  dnvm = new BankedMemory::BankedMemory("dnvm", NVRAM_START, DNVM_CTRL_BASE, 8,
                                        NVRAM_SIZE / 8, /*isVolatile=*/false);

  isram = new BankedMemory::BankedMemory("isram", ISRAM_START, ISRAM_CTRL_BASE,
                                         32, ISRAM_SIZE / 32);
  dsram = new BankedMemory::BankedMemory("dsram", DSRAM_START, DSRAM_CTRL_BASE,
                                         8, DSRAM_SIZE / 8);

  /* ------ Peripherals ------ */
  scb = new DummyPeripheral("scb", 0xe000ed00, 0xe000ed8f);
  sysTick = new SysTick("sysTick");
  nvic = new Nvic("nvic");
  mon = new SimpleMonitor("mon", SIMPLE_MONITOR_BASE);
  gpio0 = new Gpio("gpio0", GPIO0_BASE, GPIO0_EXCEPT_ID);
  gpio1 = new Gpio("gpio1", GPIO1_BASE, GPIO1_EXCEPT_ID);
  spi = new Spi("spi", SPI1_BASE, SPI1_BASE + SPI_SIZE - 1);
  dma = new Dma("dma", DMA_BASE);

  // Write tracker hooked up to data sram, used for tracking modified data
  writeTracker =
      new WriteTracker("writeTracker", /*startAddress=*/WRITE_TRACKER_BASE,
                       /*monitorEndAddress=*/DSRAM_SIZE - 1,
                       /*blockSize=*/WRITE_TRACKER_BLOCK_SIZE);
  dsram->analysisPort.bind(*writeTracker);

  slaves.push_back(gpio0);
  slaves.push_back(gpio1);
  slaves.push_back(dma);
  slaves.push_back(mon);
  slaves.push_back(nvic);
  slaves.push_back(scb);
  slaves.push_back(spi);
  slaves.push_back(isram);
  slaves.push_back(&isram->m_ctrl);
  slaves.push_back(dsram);
  slaves.push_back(&dsram->m_ctrl);
  slaves.push_back(sysTick);
  slaves.push_back(writeTracker);

  /* ------ PowerController ------ */

  // Set up power-on power gating
  unsigned porGating = 0xffffffff; // Everything powered on

  if (Config::get().getBool(std::string(this->name()) + ".icache.Enable")) {
    // No need for instruction sram if an instruction cache is used
    porGating &= ~PowerControllerOutputAssignment::iSram;
  }
  if (Config::get().getBool(std::string(this->name()) + ".dcache.Enable")) {
    // No need for data sram if data cache is used
    porGating &= ~PowerControllerOutputAssignment::dSram;
  }

  if (!Config::get().getBool(
          fmt::format("{}.isram.{}", this->name(), "Enable"))) {
    porGating &= ~PowerControllerOutputAssignment::iSram;
  }
  if (!Config::get().getBool(
          fmt::format("{}.dsram.{}", this->name(), "Enable"))) {
    porGating &= ~PowerControllerOutputAssignment::dSram;
  }

  powerController =
      new PowerController("powerController", POWER_CONTROLLER_BASE, porGating);

  //! powerController can't be in slaves, because it needs a different reset
  //! input signal
  powerController->powerModelPort.bind(powerModelPort);
  powerController->systemClk.bind(masterClock);
  powerController->pwrOn.bind(nReset);
  bus.bindTarget(static_cast<BusTarget &>(*powerController));

  /* ------ Set up Instruction and data cache according to config ------ */
  if (Config::get().getBool(
          fmt::format("{}.icache.{}", this->name(), "Enable"))) {
    icache = new Cache("icache", ROM_START, ROM_START + ROM_SIZE - 1,
                       ICACHE_CTRL_BASE);
    icache->iSocket.bind(invm->tSocket);

    invm->pwrOn.bind(nReset);
    invm->systemClk.bind(masterClock);
    invm->powerModelPort.bind(powerModelPort);

    slaves.push_back(&invm->m_ctrl);
    slaves.push_back(icache);
    slaves.push_back(&icache->cacheCtrl);
  } else {
    slaves.push_back(invm);
    slaves.push_back(&invm->m_ctrl);
  }

  if (Config::get().getBool(
          fmt::format("{}.dcache.{}", this->name(), "Enable"))) {
    dcache = new Cache("dcache", NVRAM_START, NVRAM_START + NVRAM_SIZE - 1,
                       DCACHE_CTRL_BASE);

    undoLogger =
        new UndoLogger("undoLogger", /*memBaseAddress=*/NVRAM_START,
                       /*ctrlBaseAddress=*/UNDO_LOGGER_BASE,
                       Config::get().getUint(std::string(this->name()) +
                                             ".undoLogger.capacity"),
                       Config::get().getUint(std::string(this->name()) +
                                             ".dcache.CacheLineWidth"),
                       UNDO_LOGGER_EXCEPT_ID);

    dcache->iSocket.bind(undoLogger->memSocket);
    undoLogger->iSocket.bind(dnvm->tSocket);

    dnvm->pwrOn.bind(nReset);
    dnvm->systemClk.bind(masterClock);
    dnvm->powerModelPort.bind(powerModelPort);

    undoLogger->irq.bind(nvic_irq[UNDO_LOGGER_EXCEPT_ID]);
    undoLogger->dmaTrigger.bind(dmaTrigger[UNDO_LOGGER_DMA_TRIGGER_CHANNEL]);
    undoLogger->returning_exception.bind(cpu_returning_exception);

    slaves.push_back(&dnvm->m_ctrl);
    slaves.push_back(dcache);
    slaves.push_back(&dcache->cacheCtrl);
    slaves.push_back(undoLogger);
  } else {
    slaves.push_back(dnvm);
    slaves.push_back(&dnvm->m_ctrl);
  }

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
  gpio0->clk.bind(peripheralClock);
  gpio1->clk.bind(peripheralClock);

  // Interrupts
  m_cpu.activeException.bind(cpu_active_exception);
  m_cpu.returningException.bind(cpu_returning_exception);
  m_cpu.sysTickIrq.bind(systick_irq);
  m_cpu.nvicIrq.bind(nvic_pending);

  sysTick->irq.bind(systick_irq);
  sysTick->returning_exception.bind(cpu_returning_exception);

  spi->irq.bind(nvic_irq[SPI1_EXCEPT_ID]);
  spi->active_exception.bind(cpu_active_exception);

  gpio0->irq.bind(nvic_irq[GPIO0_EXCEPT_ID]);
  gpio0->active_exception.bind(cpu_active_exception);

  gpio1->irq.bind(nvic_irq[GPIO1_EXCEPT_ID]);
  gpio1->active_exception.bind(cpu_active_exception);

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
  for (int i = 0; i < nResetGated.size(); ++i) {
    powerController->out[i].bind(nResetGated[i]);
  }

  m_cpu.pwrOn.bind(nReset);
  for (int i = 0; i < slaves.size(); ++i) {
    slaves[i]->pwrOn.bind(nResetGated[i]);
    spdlog::info("PowerController:: connecting {:s} to pin {:d}",
                 slaves[i]->name(), i);
  }

  // Events for power model
  m_cpu.powerModelPort.bind(powerModelPort);
  for (const auto &s : slaves) {
    s->powerModelPort.bind(powerModelPort);
  }

  // Miscellaneous
  // invm->waitStates.bind(nvmWaitStates);
  // dnvm->waitStates.bind(nvmWaitStates);
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

MemicMicrocontroller::~MemicMicrocontroller() {
  delete scb;
  delete gpio0;
  delete gpio1;
  delete dma;
  delete mon;
  delete nvic;
  delete spi;
  delete isram;
  delete dsram;
  delete sysTick;
  delete writeTracker;
  delete icache;
  delete dcache;
  delete invm;
  delete dnvm;
  delete powerController;
  delete undoLogger;
}

bool MemicMicrocontroller::dbgReadMem(uint8_t *out, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;

  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(out);
  trans.set_command(tlm::TLM_READ_COMMAND);
  return bus.transport_dbg(0, trans);
}

bool MemicMicrocontroller::dbgWriteMem(uint8_t *src, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;
  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(src);
  trans.set_command(tlm::TLM_WRITE_COMMAND);

  return bus.transport_dbg(0, trans);
}

std::ostream &operator<<(std::ostream &os, const MemicMicrocontroller &rhs) {
  os << "<MemicMicrocontroller> " << rhs.name() << "\n";
  os << "Memory map:\n"
     << "address(start)    address(end)  Name\n";
  for (unsigned int i = 0; i < rhs.slaves.size(); i++) {
    os << fmt::format("0x{:08x}        0x{:08x}    {:s}\n",
                      rhs.slaves[i]->startAddress(),
                      rhs.slaves[i]->endAddress(), rhs.slaves[i]->name());
  }
  return os;
}
