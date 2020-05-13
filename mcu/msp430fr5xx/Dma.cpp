/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include "mcu/BusTarget.hpp"
#include "mcu/msp430fr5xx/Dma.hpp"

extern "C" {
#include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;
using namespace tlm;

Dma::Dma(const sc_core::sc_module_name name, const sc_core::sc_time delay)
    : BusTarget(name, DMA_BASE, DMA_BASE + 0xb, delay) {
  // Construct & bind submodules
  for (auto i = 0; i < NCHANNELS; i++) {
    m_channels[i] = new DmaChannel(fmt::format("ch{:d}", i).c_str());
    m_channels[i]->clk.bind(clk);
    m_channels[i]->pending.bind(m_channelPending[i]);
    m_channels[i]->accept.bind(m_channelAccept[i]);
  }

  const unsigned undef = 0xAAAA;
  // Build register file
  // Control registers
  m_regs.addRegister(OFS_DMACTL0, 0);
  m_regs.addRegister(OFS_DMACTL1, 0);
  m_regs.addRegister(OFS_DMACTL2, 0);
  m_regs.addRegister(OFS_DMACTL4, 0);
  m_regs.addRegister(OFS_DMAIV, 0, RegisterFile::READ);
  m_regs.addRegister(OFS_DMA0CTL, 0);
  m_regs.addRegister(OFS_DMA0SA, undef);
  m_regs.addRegister(OFS_DMA0DA, undef);
  m_regs.addRegister(OFS_DMA0SZ, undef);
  m_regs.addRegister(OFS_DMA1CTL, 0);
  m_regs.addRegister(OFS_DMA1SA, undef);
  m_regs.addRegister(OFS_DMA1DA, undef);
  m_regs.addRegister(OFS_DMA1SZ, undef);
  m_regs.addRegister(OFS_DMA2CTL, 0);
  m_regs.addRegister(OFS_DMA2SA, undef);
  m_regs.addRegister(OFS_DMA2DA, undef);
  m_regs.addRegister(OFS_DMA2SZ, undef);
  m_regs.addRegister(OFS_DMA3CTL, 0);
  m_regs.addRegister(OFS_DMA3SA, undef);
  m_regs.addRegister(OFS_DMA3DA, undef);
  m_regs.addRegister(OFS_DMA3SZ, undef);
  m_regs.addRegister(OFS_DMA4CTL, 0);
  m_regs.addRegister(OFS_DMA4SA, undef);
  m_regs.addRegister(OFS_DMA4DA, undef);
  m_regs.addRegister(OFS_DMA4SZ, undef);
  m_regs.addRegister(OFS_DMA5CTL, 0);
  m_regs.addRegister(OFS_DMA5SA, undef);
  m_regs.addRegister(OFS_DMA5DA, undef);
  m_regs.addRegister(OFS_DMA5SZ, undef);

  // Set up methods & threads
  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_METHOD(interruptUpdate);
  sensitive << m_updateIrqEvent;

  SC_THREAD(process);
}

void Dma::reset() {
  // reset register values to defaults, or 0xAAAA if undefined
  const unsigned undef = 0xAAAA;
  m_regs.write(OFS_DMACTL0, 0, /*force=*/true);
  m_regs.write(OFS_DMACTL1, 0, /*force=*/true);
  m_regs.write(OFS_DMACTL2, 0, /*force=*/true);
  m_regs.write(OFS_DMACTL4, 0, /*force=*/true);
  m_regs.write(OFS_DMAIV, 0, /*force=*/true);
  m_regs.write(OFS_DMA0CTL, 0, /*force=*/true);
  m_regs.write(OFS_DMA0SA, undef, /*force=*/true);
  m_regs.write(OFS_DMA0DA, undef, /*force=*/true);
  m_regs.write(OFS_DMA0SZ, undef, /*force=*/true);
  m_regs.write(OFS_DMA1CTL, 0, /*force=*/true);
  m_regs.write(OFS_DMA1SA, undef, /*force=*/true);
  m_regs.write(OFS_DMA1DA, undef, /*force=*/true);
  m_regs.write(OFS_DMA1SZ, undef, /*force=*/true);
  m_regs.write(OFS_DMA2CTL, 0, /*force=*/true);
  m_regs.write(OFS_DMA2SA, undef, /*force=*/true);
  m_regs.write(OFS_DMA2DA, undef, /*force=*/true);
  m_regs.write(OFS_DMA2SZ, undef, /*force=*/true);
  m_regs.write(OFS_DMA3CTL, 0, /*force=*/true);
  m_regs.write(OFS_DMA3SA, undef, /*force=*/true);
  m_regs.write(OFS_DMA3DA, undef, /*force=*/true);
  m_regs.write(OFS_DMA3SZ, undef, /*force=*/true);
  m_regs.write(OFS_DMA4CTL, 0, /*force=*/true);
  m_regs.write(OFS_DMA4SA, undef, /*force=*/true);
  m_regs.write(OFS_DMA4DA, undef, /*force=*/true);
  m_regs.write(OFS_DMA4SZ, undef, /*force=*/true);
  m_regs.write(OFS_DMA5CTL, 0, /*force=*/true);
  m_regs.write(OFS_DMA5SA, undef, /*force=*/true);
  m_regs.write(OFS_DMA5DA, undef, /*force=*/true);
  m_regs.write(OFS_DMA5SZ, undef, /*force=*/true);

  // Reset channels
  for (auto &c : m_channels) {
    c->setTrigger(trigger[0].posedge_event());
    c->reset();
  }
}

void Dma::b_transport(tlm::tlm_generic_payload &trans,
                      sc_core::sc_time &delay) {
  BusTarget::b_transport(trans, delay);
  auto addr = trans.get_address();
  auto val = m_regs.read(addr);

  // Utility for setting the trigger source for a channel
  auto setTrigger = [=](DmaChannel *ch, int n) {
    if (n < 30) {
      ch->setTrigger(trigger[n].posedge_event());
    } else if (n == 30) {
      spdlog::warn("Internal DMA triggers not yet implemented.");
      sc_abort();
    } else {
      spdlog::error("DMAE0 trigger not yet implemented");
      sc_abort();
    }
  };

  if (trans.get_command() == TLM_WRITE_COMMAND) {
    switch (addr) {
      case OFS_DMACTL0:
        setTrigger(m_channels[0], val & 0x1f);
        setTrigger(m_channels[1], (val & (0x1f << 8)) >> 8);
        break;
      case OFS_DMACTL1:
        setTrigger(m_channels[2], val & 0x1f);
        setTrigger(m_channels[3], (val & (0x1f << 8)) >> 8);
        break;
      case OFS_DMACTL2:
        setTrigger(m_channels[4], val & 0x1f);
        setTrigger(m_channels[5], (val & (0x1f << 8)) >> 8);
        break;
      case OFS_DMACTL4:
        spdlog::warn("{}: DMACTL4 functionality is not implemented");
        break;
      case OFS_DMA0CTL:
        m_channels[0]->updateConfig(val);
        break;
      case OFS_DMA1CTL:
        m_channels[1]->updateConfig(val);
        break;
      case OFS_DMA2CTL:
        m_channels[2]->updateConfig(val);
        break;
      case OFS_DMA3CTL:
        m_channels[3]->updateConfig(val);
        break;
      case OFS_DMA4CTL:
        m_channels[4]->updateConfig(val);
        break;
      case OFS_DMA5CTL:
        m_channels[5]->updateConfig(val);
        break;
    }
  } else if (trans.get_command() == TLM_READ_COMMAND) {
  }

  if (addr == OFS_DMAIV) {
    // Clear pending and set next pending
    m_clearIfg = true;
    m_updateIrqEvent.notify(delay);
  } else if (addr >= OFS_DMA0SA && addr <= OFS_DMA5SZ) {
    updateChannelAddresses();
  }
}

void Dma::process() {
  uint8_t data[2];
  sc_time delay = SC_ZERO_TIME;
  tlm_generic_payload trans;  //! Outgoing transaction
  trans.set_data_ptr(data);
  wait(SC_ZERO_TIME);  // Wait for simulation start

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
      wait(clk->getPeriod());

      // Accept, perform transfer, update state & registers
      const auto &ch = *m_channels[channelIdx];
      const auto offset = channelIdx * (OFS_DMA1CTL - OFS_DMA0CTL);
      trans.set_data_length((ch.sourceBytes == DmaChannel::Bytes::Byte) ? 1
                                                                        : 2);
      m_regs.clearBitMask(OFS_DMA0CTL + offset, DMAREQ);
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
        m_regs.write(OFS_DMA0SZ + offset, ch.size);
      }

      if (!ch.enable) {
        m_regs.clearBitMask(OFS_DMA0CTL + offset, DMAEN);
      }
      if (ch.interruptFlag && ch.interruptEnable) {
        m_updateIrqEvent.notify(SC_ZERO_TIME);
      }
    } else {
      wait(waitfor);
    }
  }
}

void Dma::interruptUpdate() {
  int highestPrioChannel = -1;
  for (auto i = 0; i < m_channels.size(); i++) {
    const auto offset = i * (OFS_DMA1CTL - OFS_DMA0CTL);
    if (m_channels[i]->interruptFlag) {
      m_regs.setBitMask(OFS_DMA0CTL + offset, DMAIFG);
      highestPrioChannel = (highestPrioChannel == -1) ? i : highestPrioChannel;
    } else {
      m_regs.clearBitMask(OFS_DMA0CTL + offset, DMAIFG);
    }
  }

  if (m_clearIfg && (highestPrioChannel != -1)) {
    // Clear highest priority pending flag & re-update interrupt state
    m_channels[highestPrioChannel]->interruptFlag = false;
    m_updateIrqEvent.notify(SC_ZERO_TIME);
    return;
  }
  m_clearIfg = false;

  // Set IV & irq
  if (highestPrioChannel > -1) {
    m_regs.write(OFS_DMAIV, 1u << (highestPrioChannel + 1), /*force=*/true);
    if (m_channels[highestPrioChannel]->interruptEnable) {
      irq.write(true);
    }
  } else {
    m_regs.write(OFS_DMAIV, 0, /*force=*/true);
    irq.write(false);
  }
}

void DmaChannel::updateAddresses() {
  switch (destinationAutoIncrement) {
    case AutoIncrementMode::Unchanged:
      break;
    case AutoIncrementMode::Decrement:
      m_tDestinationAddress -= (destinationBytes == Bytes::Word) ? 2 : 1;
      break;
    case AutoIncrementMode::Increment:
      m_tDestinationAddress += (destinationBytes == Bytes::Word) ? 2 : 1;
      break;
  }
  switch (sourceAutoIncrement) {
    case AutoIncrementMode::Unchanged:
      break;
    case AutoIncrementMode::Decrement:
      m_tSourceAddress -= (sourceBytes == Bytes::Word) ? 2 : 1;
      break;
    case AutoIncrementMode::Increment:
      m_tSourceAddress += (sourceBytes == Bytes::Word) ? 2 : 1;
      break;
  }
}

void DmaChannel::updateConfig(const unsigned cfg) {
  interruptEnable = cfg & DMAIE;
  enable = cfg & DMAEN;
  levelSensitive = cfg & DMALEVEL;
  sourceBytes = (cfg & DMASRCBYTE) ? Bytes::Byte : Bytes::Word;
  destinationBytes = (cfg & DMADSTBYTE) ? Bytes::Byte : Bytes::Word;

  switch ((cfg & DMASRCINCR) >> 8) {
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

  switch ((cfg & DMADSTINCR) >> 10) {
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

  switch ((cfg & DMADT) >> 12) {
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
  if (cfg & DMAREQ) {
    m_softwareTrigger.notify(SC_ZERO_TIME);
  }
}

void DmaChannel::setTrigger(const sc_event &e) {
  m_trigger = sc_event_or_list(e);
}

void DmaChannel::process() {
  wait(SC_ZERO_TIME);  // Wait for simulation to start

  while (true) {
    wait(m_trigger | m_softwareTrigger);
    while (!enable) {
      wait(m_trigger | m_softwareTrigger);
    }

    switch (transferMode) {
      case TransferMode::Single:
        // One transfer per trigger
        if (size == m_tSize) {  // First transfer ->load internal registers
          m_tSourceAddress = sourceAddress;
          m_tDestinationAddress = destinationAddress;
        }
        pending.write(true);
        wait(accept.posedge_event());
        updateAddresses();
        if (size > 1) {  // Continue
          size--;
        } else {  // Job done, reset size
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
        pending.write(true);
        while (size) {
          std::cout << "Block-transfer: remaining " << size << std::endl;
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
        pending.write(true);
        while (size) {
          wait(accept.posedge_event());
          updateAddresses();
          if ((size > 0) && (size < m_tSize) && (size % 4 == 0)) {
            wait(2 * clk->getPeriod());
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
        if (size == m_tSize) {  // First transfer ->load internal registers
          m_tSourceAddress = sourceAddress;
          m_tDestinationAddress = destinationAddress;
        }
        pending.write(true);
        wait(accept.posedge_event());
        updateAddresses();
        if (size > 1) {
          size--;
        } else {  // Job done, reset size
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
        pending.write(true);
        while (size) {
          wait(accept.posedge_event());
          updateAddresses();
          if (!enable) {
            break;
          }
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
          while (size) {
            wait(accept.posedge_event());
            updateAddresses();
            if ((size > 0) && (size < m_tSize) && (size % 4 == 0)) {
              wait(2 * clk->getPeriod());
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
    const auto offset = i * (OFS_DMA1CTL - OFS_DMA0CTL);
    m_channels[i]->sourceAddress = m_regs.read(OFS_DMA0SA + offset);
    m_channels[i]->destinationAddress = m_regs.read(OFS_DMA0DA + offset);
    m_channels[i]->size = m_regs.read(OFS_DMA0SZ + offset);
    if (!m_channels[i]->enable) {
      m_channels[i]->m_tSize = m_regs.read(OFS_DMA0SZ + offset);
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
