/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <stdint.h>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <systemc>
#include <tlm>
#include "include/fused.h"
#include "mcu/Cm0Microcontroller.hpp"
#include "mcu/Microcontroller.hpp"
#include "mcu/cortex-m0/CortexM0Cpu.hpp"
#include "mcu/cortex-m0/Nvic.hpp"
#include "mcu/cortex-m0/OutputPort.hpp"
#include "mcu/cortex-m0/SysTick.hpp"
#include "utilities/Config.hpp"

using namespace sc_core;

Cm0Microcontroller::Cm0Microcontroller(sc_module_name nm)
    : Microcontroller(nm), m_cpu("CPU", m_cycleTime), bus("bus") {
  for (unsigned i = 0; i < nvic_irq.size(); i++) {
    nvic_irq[i].write(false);
  }
  /* ------ Memories ------ */
  invm = new NonvolatileMemory("invm", ROM_START, ROM_START + ROM_SIZE - 1,
                               m_cycleTime);
  dnvm = new NonvolatileMemory("dnvm", NVRAM_START,
                               NVRAM_START + NVRAM_SIZE - 1, m_cycleTime);

  sram = new VolatileMemory("sram", SRAM_START, SRAM_START + SRAM_SIZE - 1,
                            m_cycleTime);

  /* ------ Peripherals ------ */
  scb = new DummyPeripheral("scb", 0xe000ed00, 0xe000ed8f, m_cycleTime);
  sysTick = new SysTick("sysTick", m_cycleTime);
  nvic = new Nvic("nvic", m_cycleTime);
  mon = new SimpleMonitor("mon", m_cycleTime);
  outputPort = new OutputPort("outputPort", m_cycleTime);

  slaves.push_back(sram);
  slaves.push_back(mon);
  slaves.push_back(sysTick);
  slaves.push_back(nvic);
  slaves.push_back(scb);
  slaves.push_back(outputPort);
  slaves.push_back(invm);
  slaves.push_back(dnvm);

  /* ------------------------ */

  // Sort slaves by address
  std::sort(slaves.begin(), slaves.end(), [](BusTarget *a, BusTarget *b) {
    return a->startAddress() < b->startAddress();
  });

  /* ------ Bind ------ */

  // IO

  // Clocks
  sysTick->clk.bind(masterClock);
  nvic->clk.bind(masterClock);

  // Interrupts
  m_cpu.activeException.bind(cpu_active_exception);
  m_cpu.returningException.bind(cpu_returning_exception);
  m_cpu.sysTickIrq.bind(systick_irq);
  m_cpu.nvicIrq.bind(nvic_pending);

  sysTick->irq.bind(systick_irq);
  sysTick->returning_exception.bind(cpu_returning_exception);
  nvic->pending.bind(nvic_pending);
  nvic->returning.bind(cpu_returning_exception);
  nvic->active.bind(cpu_active_exception);

  // Bind external interrupt signals to NVIC[0:15]
  for (unsigned i = 0; i < externalIrq.size(); i++) {
    nvic->irq[i].bind(externalIrq[i]);
  }

  // Bind internal interrupt signals to NVIC[16:31]
  for (unsigned i = 0; i < nvic_irq.size(); i++) {
    nvic->irq[i + 16].bind(nvic_irq[i]);
  }

  // Reset
  m_cpu.pwrOn.bind(nReset);
  for (const auto &s : slaves) {
    s->pwrOn.bind(nReset);
  }

  // Write default const value for now.
  // vref.write(2.0);

  // Miscellaneous
  invm->waitStates.bind(nvmWaitStates);
  dnvm->waitStates.bind(nvmWaitStates);

  // Bus
  bus.addInitiator();
  m_cpu.iSocket.bind(*bus.tSockets[0]);
  for (const auto &s : slaves) {
    auto port = bus.addTarget(*s);
    s->setBusSocket(port);
    bus.iSockets[port]->bind(s->tSocket);
  }
}

bool Cm0Microcontroller::dbgReadMem(uint8_t *out, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;

  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(out);
  trans.set_command(tlm::TLM_READ_COMMAND);
  unsigned int received = bus.transport_dbg(trans);

  return received;
}

bool Cm0Microcontroller::dbgWriteMem(uint8_t *src, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;
  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(src);
  trans.set_command(tlm::TLM_WRITE_COMMAND);

  return bus.transport_dbg(trans);
}
