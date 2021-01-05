/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include "include/msp430-fused.h"
#include "mcu/Cache.hpp"
#include "mcu/Microcontroller.hpp"
#include "mcu/Msp430Microcontroller.hpp"
#include "mcu/msp430fr5xx/Msp430Cpu.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;

Msp430Microcontroller::Msp430Microcontroller(sc_module_name nm)
    : Microcontroller(nm), m_cpu("CPU", false, false), bus("bus") {
  /* ------ Memories ------ */
  cache = new Cache("cache", FRAM_START, 0xff7f);
  fram = new NonvolatileMemory("fram", FRAM_START, 0xff7f);
  vectors = new GenericMemory("vectors", 0xff80, 0xffff);
  sram = new VolatileMemory("sram", RAM_START, RAM_START + 0x2000 - 1);

  /* ------ Peripherals ------ */
  std::vector<unsigned char> zeroRetval(0x800, 0);    // Read reg's as 0s
  std::vector<unsigned char> refgenRetVal(0x800, 0);  // Read most reg's as 0s
  refgenRetVal[OFS_REFCTL0 + 1] = static_cast<uint8_t>(REFGENRDY >> 8);
  refgenRetVal[OFS_REFCTL0] = static_cast<uint8_t>(REFGENRDY & 0xff);

  pmm = new PowerManagementModule("pmm");
  adc = new Adc12("Adc");
  refgen =
      new DummyPeripheral("refgen", refgenRetVal, REF_A_BASE, REF_A_BASE + 1);
  fram_ctl = new Frctl_a("FRAM_CTL_A");
  watchdog =
      new DummyPeripheral("watchdog", zeroRetval, WDT_A_BASE, WDT_A_BASE + 1);
  mon = new SimpleMonitor("mon", SIMPLE_MONITOR_BASE);
  portJ = new DummyPeripheral("portJ", zeroRetval, PJ_BASE, PJ_BASE + 0x16);
  portA = new DigitalIo("portA", PA_BASE, PA_BASE + 0x1f);
  portB = new DigitalIo("portB", PB_BASE, PB_BASE + 0x1f);
  portC = new DigitalIo("portC", PC_BASE, PC_BASE + 0x1f);
  portD = new DigitalIo("portD", PD_BASE, PD_BASE + 0x1f);
  cs = new ClockSystem("cs", CS_BASE);
  tima = new TimerA("tima", TA0_BASE);
  interruptArbiter = new InterruptArbiter<37>("interruptArbiter", false);
  mpy32 = new Mpy32("mpy32", MPY32_BASE, MPY32_BASE + 0x2f);
  euscib = new eUSCI_B("eUSCI_B", EUSCI_B0_BASE, EUSCI_B0_BASE + 0x2f);
  dma = new Dma("Dma");

  slaves.push_back(cache);
  slaves.push_back(fram_ctl);
  slaves.push_back(sram);
  slaves.push_back(vectors);
  slaves.push_back(adc);
  slaves.push_back(refgen);
  slaves.push_back(watchdog);
  slaves.push_back(portJ);
  slaves.push_back(portA);
  slaves.push_back(portB);
  slaves.push_back(portC);
  slaves.push_back(portD);
  slaves.push_back(pmm);
  slaves.push_back(cs);
  slaves.push_back(tima);
  slaves.push_back(mpy32);
  slaves.push_back(dma);
  slaves.push_back(mon);
  slaves.push_back(euscib);

  // Sort slaves by address
  std::sort(slaves.begin(), slaves.end(), [](BusTarget *a, BusTarget *b) {
    return a->startAddress() < b->startAddress();
  });

  /* ------ Bind ------ */

  // Clocks
  m_cpu.mclk.bind(mclk);
  for (const auto &s : slaves) {
    s->systemClk.bind(mclk);
  }
  fram->systemClk.bind(mclk);

  cs->aclk.bind(aclk);
  cs->smclk.bind(smclk);
  cs->mclk.bind(mclk);
  cs->vloclk.bind(vloclk);
  cs->modclk.bind(modclk);

  tima->aclk.bind(aclk);
  tima->smclk.bind(smclk);

  adc->modclk.bind(modclk);
  adc->aclk.bind(aclk);
  adc->mclk.bind(mclk);
  adc->smclk.bind(smclk);

  euscib->aclk.bind(aclk);
  euscib->smclk.bind(smclk);

  // Interrupts
  m_cpu.ira.bind(cpu_ira);
  m_cpu.irq.bind(cpu_irq);
  m_cpu.irqIdx.bind(cpu_irqIdx);
  m_cpu.iraConnected.bind(m_iraConnected);
  interruptArbiter->iraConnected.bind(m_iraConnected);
  interruptArbiter->irqOut.bind(cpu_irq);
  interruptArbiter->iraIn.bind(cpu_ira);
  interruptArbiter->idxOut.bind(cpu_irqIdx);

  tima->ira.bind(tima_ira);
  tima->irq.bind(tima_irq);
  interruptArbiter->irqIn[10].bind(tima_irq);
  interruptArbiter->iraOut[10].bind(tima_ira);

  pmm->ira.bind(pmm_ira);
  pmm->irq.bind(pmm_irq);
  interruptArbiter->irqIn[0].bind(pmm_irq);
  interruptArbiter->iraOut[0].bind(pmm_ira);

  euscib->irq.bind(euscib_irq);
  euscib->ira.bind(euscib_ira);
  interruptArbiter->irqIn[8].bind(euscib_irq);
  interruptArbiter->iraOut[8].bind(euscib_ira);
  euscib->dmaTrigger.bind(dma_dummy);

  dma->irq.bind(dma_irq);
  dma->ira.bind(dma_ira);
  interruptArbiter->irqIn[13].bind(dma_irq);
  interruptArbiter->iraOut[13].bind(dma_ira);

  adc->irq.bind(adc_irq);
  interruptArbiter->irqIn[9].bind(adc_irq);

  portA->irq[0].bind(port1_irq);
  portA->irq[1].bind(port2_irq);
  portB->irq[0].bind(port3_irq);
  portB->irq[1].bind(port4_irq);
  portC->irq[0].bind(port5_irq);
  portC->irq[1].bind(port6_irq);
  portD->irq[0].bind(port7_irq);
  portD->irq[1].bind(port8_irq);
  interruptArbiter->irqIn[16].bind(port1_irq);
  interruptArbiter->irqIn[19].bind(port2_irq);
  interruptArbiter->irqIn[22].bind(port3_irq);
  interruptArbiter->irqIn[23].bind(port4_irq);
  interruptArbiter->irqIn[28].bind(port5_irq);
  interruptArbiter->irqIn[29].bind(port6_irq);
  interruptArbiter->irqIn[35].bind(port7_irq);
  interruptArbiter->irqIn[36].bind(port8_irq);

  // Power
  pmm->staticPower.bind(staticPower);

  // Reset
  m_cpu.pwrOn.bind(nReset);
  for (const auto &s : slaves) {
    s->pwrOn.bind(nReset);
  }
  fram->pwrOn.bind(nReset);

  // Events for power model
  m_cpu.powerModelPort.bind(powerModelPort);
  fram->powerModelPort.bind(powerModelPort);
  for (const auto &s : slaves) {
    s->powerModelPort.bind(powerModelPort);
  }

  // Analog signals
  adc->vcc.bind(vcc);
  adc->vref.bind(vref);
  pmm->vcc.bind(vcc);

  // Write default const value for now.
  vref.write(2.0);

  // Miscellaneous
  fram_ctl->waitStates.bind(framWaitStates);
  fram->waitStates.bind(framWaitStates);
  m_cpu.busStall.bind(cpuStall);

  // DMA
  dma->stallCpu.bind(cpuStall);
  tima->dmaTrigger.bind(dmaTrigger[1]);
  for (size_t i = 0; i < dmaTrigger.size(); ++i) {
    dma->trigger[i].bind(dmaTrigger[i]);
  }

  // Bus
  m_cpu.iSocket.bind(bus.tSocket);
  dma->iSocket.bind(bus.tSocket);
  for (const auto &s : slaves) {
    bus.bindTarget(*s);
  }

  // Fram and cache
  cache->iSocket.bind(fram->tSocket);
}

bool Msp430Microcontroller::dbgReadMem(uint8_t *out, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;

  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(out);
  trans.set_command(tlm::TLM_READ_COMMAND);
  return bus.transport_dbg(0, trans);
}

bool Msp430Microcontroller::dbgWriteMem(uint8_t *src, size_t addr, size_t len) {
  tlm::tlm_generic_payload trans;

  trans.set_address(addr);
  trans.set_data_length(len);
  trans.set_data_ptr(src);
  trans.set_command(tlm::TLM_WRITE_COMMAND);
  return bus.transport_dbg(0, trans);
}
