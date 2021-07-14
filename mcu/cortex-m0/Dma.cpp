/*
 * Copyright (c) 2019-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "include/cm0-fused.h"
#include "mcu/BusTarget.hpp"
#include "mcu/cortex-m0/Dma.hpp"
#include <algorithm>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

using namespace sc_core;
using namespace tlm;
using namespace CortexM0Peripherals;

Dma::Dma(const sc_module_name name, const unsigned startAddress)
    : BusTarget(name, startAddress, startAddress + 0x73) {
  // Construct & bind submodules
  for (int i = 0; i < NCHANNELS; i++) {
    m_triggerMuxes[i] =
        new TriggerMux(fmt::format("ch{:d}_trigger_mux", i).c_str());
    for (size_t j = 0; j < trigger.size(); ++j) {
      m_triggerMuxes[i]->in[j].bind(trigger[j]);
    }
    m_triggerMuxes[i]->out.bind(m_channelTrigger[i]);
    m_triggerMuxes[i]->select.bind(m_channelTriggerSelect[i]);
    m_triggerMuxes[i]->nreset.bind(pwrOn);

    m_channels[i] = new DmaChannel(fmt::format("ch{:d}", i).c_str());
    m_channels[i]->systemClk.bind(systemClk);
    m_channels[i]->pending.bind(m_channelPending[i]);
    m_channels[i]->accept.bind(m_channelAccept[i]);
    m_channels[i]->trigger.bind(m_channelTrigger[i]);
  }

  const unsigned undef = 0xAAAA;
  // Build register file
  m_regs.addRegister(RegisterAddress::DMACTL0, 0);
  m_regs.addRegister(RegisterAddress::DMACTL1, 0);
  m_regs.addRegister(RegisterAddress::DMACTL2, 0);
  m_regs.addRegister(RegisterAddress::DMACTL4, 0);
  m_regs.addRegister(RegisterAddress::DMAIV, 0);
  m_regs.addRegister(RegisterAddress::DMA0CTL, 0);
  m_regs.addRegister(RegisterAddress::DMA0SA, undef);
  m_regs.addRegister(RegisterAddress::DMA0DA, undef);
  m_regs.addRegister(RegisterAddress::DMA0SZ, undef);
  m_regs.addRegister(RegisterAddress::DMA1CTL, 0);
  m_regs.addRegister(RegisterAddress::DMA1SA, undef);
  m_regs.addRegister(RegisterAddress::DMA1DA, undef);
  m_regs.addRegister(RegisterAddress::DMA1SZ, undef);
  m_regs.addRegister(RegisterAddress::DMA2CTL, 0);
  m_regs.addRegister(RegisterAddress::DMA2SA, undef);
  m_regs.addRegister(RegisterAddress::DMA2DA, undef);
  m_regs.addRegister(RegisterAddress::DMA2SZ, undef);
  m_regs.addRegister(RegisterAddress::DMA3CTL, 0);
  m_regs.addRegister(RegisterAddress::DMA3SA, undef);
  m_regs.addRegister(RegisterAddress::DMA3DA, undef);
  m_regs.addRegister(RegisterAddress::DMA3SZ, undef);
  m_regs.addRegister(RegisterAddress::DMA4CTL, 0);
  m_regs.addRegister(RegisterAddress::DMA4SA, undef);
  m_regs.addRegister(RegisterAddress::DMA4DA, undef);
  m_regs.addRegister(RegisterAddress::DMA4SZ, undef);
  m_regs.addRegister(RegisterAddress::DMA5CTL, 0);
  m_regs.addRegister(RegisterAddress::DMA5SA, undef);
  m_regs.addRegister(RegisterAddress::DMA5DA, undef);
  m_regs.addRegister(RegisterAddress::DMA5SZ, undef);

  // Set up methods & threads
  SC_HAS_PROCESS(Dma);
  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_METHOD(interruptUpdate);
  sensitive << m_updateIrqFlagEvent;

  SC_METHOD(irqControl);
  sensitive << active_exception << m_updateIrqEvent;
  dont_initialize();

  SC_THREAD(process);
}

void Dma::reset() {
  m_regs.reset();

  // Reset channels
  for (size_t i = 0; i < NCHANNELS; ++i) {
    m_channels[i]->reset();
  }
}

void Dma::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  BusTarget::b_transport(trans, delay);
  auto addr = trans.get_address();
  auto val = m_regs.read(addr);

  // Utility for setting the trigger source for a channel
  auto setTrigger = [=](const int channelNumber, int triggerNumber) {
    if (triggerNumber < 30) {
      m_channelTriggerSelect[channelNumber].write(triggerNumber);
    } else if (triggerNumber == 30) {
      spdlog::warn("Internal DMA triggers not yet implemented.");
      sc_abort();
    } else {
      spdlog::error("DMAE0 trigger not yet implemented");
      sc_abort();
    }
    spdlog::info("{:s}: Channel {:d} trigger set to {:d}", this->name(),
                 channelNumber, triggerNumber);
  };

  if (trans.get_command() == TLM_WRITE_COMMAND) {
    switch (addr) {
    case RegisterAddress::DMACTL0:
      setTrigger(0, val & 0x1f);
      setTrigger(1, (val & (0x1f << 8)) >> 8);
      break;
    case RegisterAddress::DMACTL1:
      setTrigger(2, val & 0x1f);
      setTrigger(3, (val & (0x1f << 8)) >> 8);
      break;
    case RegisterAddress::DMACTL2:
      setTrigger(4, val & 0x1f);
      setTrigger(5, (val & (0x1f << 8)) >> 8);
      break;
    case RegisterAddress::DMACTL4:
      spdlog::warn("{}: DMACTL4 functionality is not implemented");
      break;
    case RegisterAddress::DMA0CTL:
      m_channels[0]->updateConfig(val);
      break;
    case RegisterAddress::DMA1CTL:
      m_channels[1]->updateConfig(val);
      break;
    case RegisterAddress::DMA2CTL:
      m_channels[2]->updateConfig(val);
      break;
    case RegisterAddress::DMA3CTL:
      m_channels[3]->updateConfig(val);
      break;
    case RegisterAddress::DMA4CTL:
      m_channels[4]->updateConfig(val);
      break;
    case RegisterAddress::DMA5CTL:
      m_channels[5]->updateConfig(val);
      break;
    }
  } else if (trans.get_command() == TLM_READ_COMMAND) {
  }

  if (addr == RegisterAddress::DMAIV) {
    // Clear pending and set next pending
    m_regs.write(RegisterAddress::DMAIV, 0, /*force=*/true);
    m_clearIfg = true;
    m_updateIrqFlagEvent.notify(delay);
  } else if (addr >= RegisterAddress::DMA0SA &&
             addr <= RegisterAddress::DMA5SZ) {
    updateChannelAddresses();
  }
}

void Dma::process() {
  uint8_t data[4];
  sc_time delay = SC_ZERO_TIME;
  tlm_generic_payload trans; //! Outgoing transaction
  trans.set_data_ptr(data);
  wait(SC_ZERO_TIME); // Wait for simulation start

  // Sensitivity list
  sc_event_or_list waitfor;
  for (auto &c : m_channelPending) {
    waitfor |= c.posedge_event();
  }

  while (true) {
    int channelIdx = -1;
    // Find highest-priority (lowest channel number) triggered channel
    for (auto i = 0; i < m_channelPending.size(); i++) {
      if (m_channelPending[i].read()) {
        channelIdx = i;
        break;
      }
    }

    if (channelIdx >= 0) {
      // DMA spends "1 or 2 clock cycles to synchronize to mclk"
      busStall.write(true);
      wait(systemClk->getPeriod());

      // Accept, perform transfer, update state & registers
      const auto &ch = *m_channels[channelIdx];
      const auto channelAddressOffset =
          channelIdx * (RegisterAddress::DMA1CTL - RegisterAddress::DMA0CTL);
      trans.set_data_length((ch.sourceBytes == DmaChannel::Bytes::Byte) ? 1
                                                                        : 4);
      m_regs.clearBitMask(RegisterAddress::DMA0CTL + channelAddressOffset,
                          BitMask::DMAREQ);
      while (ch.pending.read()) {
        // Read
        trans.set_command(TLM_READ_COMMAND);
        trans.set_address(ch.m_tSourceAddress);
        delay = SC_ZERO_TIME;
        iSocket->b_transport(trans, delay);
        wait(delay);

        // Write
        trans.set_address(ch.m_tDestinationAddress);
        trans.set_command(TLM_WRITE_COMMAND);
        m_channelAccept[channelIdx].write(true);
        delay = SC_ZERO_TIME;
        iSocket->b_transport(trans, delay);
        wait(delay);
        m_channelAccept[channelIdx].write(false);
        m_regs.write(RegisterAddress::DMA0SZ + channelAddressOffset, ch.size);
      }

      if (!ch.enable) {
        m_regs.clearBitMask(RegisterAddress::DMA0CTL + channelAddressOffset,
                            BitMask::DMAEN);
      }
      if (ch.interruptFlag && ch.interruptEnable) {
        m_updateIrqFlagEvent.notify(SC_ZERO_TIME);
      }
      busStall.write(false);
    } else {
      wait(waitfor);
    }
  }
}

void Dma::interruptUpdate() {
  int highestPrioChannel = -1;
  for (auto i = 0; i < m_channels.size(); i++) {
    const auto offset =
        i * (RegisterAddress::DMA1CTL - RegisterAddress::DMA0CTL);
    if (m_channels[i]->interruptFlag) {
      m_regs.setBitMask(RegisterAddress::DMA0CTL + offset, BitMask::DMAIFG);
      highestPrioChannel = (highestPrioChannel == -1) ? i : highestPrioChannel;
    } else {
      m_regs.clearBitMask(RegisterAddress::DMA0CTL + offset, BitMask::DMAIFG);
    }
  }

  if (m_clearIfg && (highestPrioChannel != -1)) {
    // Clear highest priority pending flag & re-update interrupt state
    m_channels[highestPrioChannel]->interruptFlag = false;
    m_updateIrqFlagEvent.notify(SC_ZERO_TIME);
    return;
  }
  m_clearIfg = false;

  // Set IV & irq
  if (highestPrioChannel > -1) {
    m_regs.write(RegisterAddress::DMAIV, 1u << (highestPrioChannel + 1),
                 /*force=*/true);
    if (m_channels[highestPrioChannel]->interruptEnable) {
      m_setIrq = true;
      m_updateIrqEvent.notify(SC_ZERO_TIME);
    }
  } else {
    m_regs.write(RegisterAddress::DMAIV, 0, /*force=*/true);
  }
}

void Dma::irqControl() {
  if (pwrOn.read() == false) {
    irq.write(false);
    m_setIrq = false;
    return;
  }
  if (m_setIrq && (!irq.read())) {
    spdlog::info("{:s}: @{:s} interrupt request", this->name(),
                 sc_time_stamp().to_string());
    irq->write(true);
  } else if ((active_exception.read() - 16) == DMA_EXCEPT_ID) {
    spdlog::info("{:s}: @{:s} interrupt request cleared.", this->name(),
                 sc_time_stamp().to_string());
    irq->write(false);

    // Clear interrupt vector flags & registers
    m_clearIfg = true;
    m_updateIrqFlagEvent.notify(SC_ZERO_TIME);
  }
  m_setIrq = false;
}

void DmaChannel::updateAddresses() {
  switch (destinationAutoIncrement) {
  case AutoIncrementMode::Unchanged:
    break;
  case AutoIncrementMode::Decrement:
    m_tDestinationAddress -= (destinationBytes == Bytes::Word) ? 4 : 1;
    break;
  case AutoIncrementMode::Increment:
    m_tDestinationAddress += (destinationBytes == Bytes::Word) ? 4 : 1;
    break;
  }
  switch (sourceAutoIncrement) {
  case AutoIncrementMode::Unchanged:
    break;
  case AutoIncrementMode::Decrement:
    m_tSourceAddress -= (sourceBytes == Bytes::Word) ? 4 : 1;
    break;
  case AutoIncrementMode::Increment:
    m_tSourceAddress += (sourceBytes == Bytes::Word) ? 4 : 1;
    break;
  }
}

void DmaChannel::updateConfig(const unsigned cfg) {
  interruptEnable = cfg & Dma::BitMask::DMAIE;
  enable = cfg & Dma::BitMask::DMAEN;
  levelSensitive = cfg & Dma::BitMask::DMALEVEL;
  sourceBytes = (cfg & Dma::BitMask::DMASRCBYTE) ? Bytes::Byte : Bytes::Word;
  destinationBytes =
      (cfg & Dma::BitMask::DMADSTBYTE) ? Bytes::Byte : Bytes::Word;

  switch ((cfg & Dma::BitMask::DMASRCINCR) >> 8) {
  case 0:
    sourceAutoIncrement = AutoIncrementMode::Unchanged;
    break;
  case 1:
    sourceAutoIncrement = AutoIncrementMode::Unchanged;
    break;
  case 2:
    sourceAutoIncrement = AutoIncrementMode::Decrement;
    break;
  case 3:
    sourceAutoIncrement = AutoIncrementMode::Increment;
    break;
  }

  switch ((cfg & Dma::BitMask::DMADSTINCR) >> 10) {
  case 0:
    destinationAutoIncrement = AutoIncrementMode::Unchanged;
    break;
  case 1:
    destinationAutoIncrement = AutoIncrementMode::Unchanged;
    break;
  case 2:
    destinationAutoIncrement = AutoIncrementMode::Decrement;
    break;
  case 3:
    destinationAutoIncrement = AutoIncrementMode::Increment;
    break;
  }

  switch ((cfg & Dma::BitMask::DMADT) >> 12) {
  case 0:
    transferMode = TransferMode::Single;
    break;
  case 1:
    transferMode = TransferMode::Block;
    break;
  case 2:
    transferMode = TransferMode::BurstBlock;
    break;
  case 3:
    transferMode = TransferMode::BurstBlock;
    break;
  case 4:
    transferMode = TransferMode::RepeatedSingle;
    break;
  case 5:
    transferMode = TransferMode::RepeatedBlock;
    break;
  case 6:
    transferMode = TransferMode::RepeatedBurstBlock;
    break;
  case 7:
    transferMode = TransferMode::RepeatedBurstBlock;
    break;
  }

  // Software trigger
  if (cfg & Dma::BitMask::DMAREQ) {
    m_softwareTrigger.notify(SC_ZERO_TIME);
  }
}

void DmaChannel::process() {
  wait(SC_ZERO_TIME); // Wait for simulation to start

  while (true) {
    wait(trigger.posedge_event() | m_softwareTrigger);
    while (!enable) {
      wait(trigger.posedge_event() | m_softwareTrigger);
    }

    switch (transferMode) {
    case TransferMode::Single:
      // One transfer per trigger
      if (size == m_tSize) { // First transfer ->load internal registers
        m_tSourceAddress = sourceAddress;
        m_tDestinationAddress = destinationAddress;
        spdlog::info(
            "{:s}: Initiated single transfer {:d} beats 0x{:08x}->0x{:08x}",
            this->name(), size, sourceAddress, destinationAddress);
      }
      pending.write(true);
      wait(accept.posedge_event());
      updateAddresses();
      if (size > 1) { // Continue
        size--;
      } else { // Job done, reset size
        size = m_tSize;
        interruptFlag = true;
        enable = false;
      }
      pending.write(false);
      break;
    case DmaChannel::TransferMode::Block:
      // Transfer whole block after first trigger
      size = m_tSize;
      m_tSourceAddress = sourceAddress;
      m_tDestinationAddress = destinationAddress;
      spdlog::info("{}: Block transfer {:d} beats 0x{:08x}->0x{:08x}",
                   this->name(), size, sourceAddress, destinationAddress);
      pending.write(true);
      while (size) {
        wait(accept.posedge_event());
        updateAddresses();
        if (!enable) {
          break;
        }
        size--;
      }
      size = m_tSize;
      enable = false;
      interruptFlag = true;
      pending.write(false);
      break;
    case DmaChannel::TransferMode::BurstBlock:
      //  CPU activity interleaved with transfer
      //  Delay for 2 cycles every 4 transfers
      size = m_tSize;
      m_tSourceAddress = sourceAddress;
      m_tDestinationAddress = destinationAddress;
      spdlog::info("{}: Block-burst transfer {:d} beats 0x{:08x}->0x{:08x}",
                   this->name(), size, sourceAddress, destinationAddress);
      pending.write(true);
      while (size) {
        wait(accept.posedge_event());
        updateAddresses();
        if ((size > 0) && (size < m_tSize) && (size % 4 == 0)) {
          wait(2 * systemClk->getPeriod());
        }
        if (!enable) {
          break;
        }
        size--;
      }
      size = m_tSize;
      interruptFlag = true;
      enable = false;
      pending.write(false);
      break;
    case TransferMode::RepeatedSingle:
      // One transfer per trigger
      if (size == m_tSize) { // First transfer ->load internal registers
        m_tSourceAddress = sourceAddress;
        m_tDestinationAddress = destinationAddress;
        spdlog::info("{}: Initiated repeated single transfer {:d} beats "
                     "0x{:08x}->0x{:08x}",
                     this->name(), size, sourceAddress, destinationAddress);
      }
      pending.write(true);
      wait(accept.posedge_event());
      updateAddresses();
      if (size > 1) {
        size--;
      } else { // Job done, reset size
        size = m_tSize;
        interruptFlag = true;
      }
      pending.write(false);
      break;
    case DmaChannel::TransferMode::RepeatedBlock:
      // Transfer one whole block after first trigger, don't clear enable
      size = m_tSize;
      m_tSourceAddress = sourceAddress;
      m_tDestinationAddress = destinationAddress;
      spdlog::info("{}: Repeated block transfer {:d} beats 0x{:08x}->0x{:08x}",
                   this->name(), size, sourceAddress, destinationAddress);
      pending.write(true);
      while (size) {
        wait(accept.posedge_event());
        updateAddresses();
        if (!enable) {
          break;
        }
        size--;
      }
      size = m_tSize;
      interruptFlag = true;
      pending.write(false);
      break;
    case DmaChannel::TransferMode::RepeatedBurstBlock:
      // Continue burst-transfer indefinitely after first trigger, until
      // breakout condition
      pending.write(true);
      while (1) {
        size = m_tSize;
        m_tSourceAddress = sourceAddress;
        m_tDestinationAddress = destinationAddress;
        spdlog::info(
            "{}: Repeated block-burst transfer {:d} beats 0x{:08x}->0x{:08x}",
            this->name(), size, sourceAddress, destinationAddress);
        while (size) {
          wait(accept.posedge_event());
          updateAddresses();
          if ((size > 0) && (size < m_tSize) && (size % 4 == 0)) {
            // Wait 2 cycles after every 4th transfer, to allow some CPU
            // activity (unstall the CPU for 2 cycles every 4 transfers)
            wait(2 * systemClk->getPeriod());
          }
          if (!enable) {
            break;
          }
          size--;
        }
        size = m_tSize;
        interruptFlag = true;
      }
      pending.write(false);
      break;
    }
  }
}

void Dma::updateChannelAddresses() {
  for (auto i = 0; i < m_channels.size(); i++) {
    const auto offset =
        i * (RegisterAddress::DMA1CTL - RegisterAddress::DMA0CTL);
    m_channels[i]->sourceAddress =
        m_regs.read(RegisterAddress::DMA0SA + offset);
    m_channels[i]->destinationAddress =
        m_regs.read(RegisterAddress::DMA0DA + offset);
    m_channels[i]->size = m_regs.read(RegisterAddress::DMA0SZ + offset);
    if (!m_channels[i]->enable) {
      m_channels[i]->m_tSize = m_regs.read(RegisterAddress::DMA0SZ + offset);
    }
  }
}

void DmaChannel::reset() {
  size = 0;
  destinationAddress = 0;
  sourceAddress = 0;
  destinationAutoIncrement = AutoIncrementMode::Unchanged;
  sourceAutoIncrement = AutoIncrementMode::Unchanged;
  destinationBytes = Bytes::Word;
  sourceBytes = Bytes::Word;
  enable = false;
  levelSensitive = false;
  abort = false;
  transferMode = TransferMode::Single;
  interruptFlag = false;
}

std::ostream &operator<<(std::ostream &os, const DmaChannel &rhs) {
  os << "<DmaChannel> " << rhs.name() << "\n";
  os << "Signals:\n";
  os << "\tpending: " << rhs.pending.read() << "\n";
  os << "Settings:\n";
  os << "\tenable: " << rhs.enable << "\n";
  os << "\tinterruptEnable: " << rhs.interruptEnable << "\n";
  os << "\tlevel sensitive: " << (rhs.levelSensitive ? "level" : "edge")
     << "\n";
  os << "\tsize: " << rhs.size << "\n";
  os << "\tdestinationAddress: 0x" << std::hex << rhs.destinationAddress
     << "\n";
  os << "\tsourceAddress: 0x" << std::hex << rhs.sourceAddress << std::dec
     << "\n";
  os << "\ttransferMode: ";

  switch (rhs.transferMode) {
  case (DmaChannel::TransferMode::Single):
    os << "single";
    break;
  case (DmaChannel::TransferMode::Block):
    os << "block";
    break;
  case (DmaChannel::TransferMode::BurstBlock):
    os << "burst-block";
    break;
  case (DmaChannel::TransferMode::RepeatedBlock):
    os << "repeated-block";
    break;
  case (DmaChannel::TransferMode::RepeatedBurstBlock):
    os << "repeated-burst-block";
    break;
  case (DmaChannel::TransferMode::RepeatedSingle):
    os << "repeated-single";
    break;
  }
  os << "\n";
  os << "\tdestinationIncrementMode: "
     << ((rhs.destinationAutoIncrement ==
          DmaChannel::AutoIncrementMode::Unchanged)
             ? "unchanged"
             : (rhs.destinationAutoIncrement ==
                DmaChannel::AutoIncrementMode::Increment)
                   ? "increment"
                   : "decrement")
     << "\n";
  os << "\tsourceIncrementMode: "
     << ((rhs.sourceAutoIncrement == DmaChannel::AutoIncrementMode::Unchanged)
             ? "unchanged"
             : (rhs.sourceAutoIncrement ==
                DmaChannel::AutoIncrementMode::Increment)
                   ? "increment"
                   : "decrement")
     << "\n";
  os << "\tdestination word size: "
     << ((rhs.destinationBytes == DmaChannel::Bytes::Byte) ? "byte" : "word")
     << "\n";
  os << "\tsource word size: "
     << ((rhs.sourceBytes == DmaChannel::Bytes::Byte) ? "byte" : "word")
     << "\n";
  os << "Variables:\n";
  os << "\tinterruptFlag: " << rhs.interruptFlag << "\n";
  os << "\ttSize: " << rhs.m_tSize << "\n";
  os << "\ttDestinationAddress: 0x" << std::hex << rhs.m_tDestinationAddress
     << std::dec << "\n";
  os << "\ttsourceAddress: 0x" << std::hex << rhs.m_tSourceAddress << std::dec
     << "\n";
  return os;
}
