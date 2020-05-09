/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include "mcu/GenericDma.hpp"

extern "C" {
include "mcu/msp430fr5xx/device_includes/msp430fr5994.h"
}

using namespace sc_core;
using namespace tlm;

GenericDma::GenericDma(const sc_core::sc_module_name name,
                       const sc_core::sc_time delay)
    : BusTarget(name, DMA_BASE, DMA_BASE + 0xb, delay) {
  // Submodules
  for (auto &c : m_channels) {
    c.clk.bind(clk);
  }

  const unsigned undef = 0xAAAA;
  // Build register file
  // Control registers
  m_regs.addRegister(OFS_DMACTL0, 0);
  m_regs.addRegister(OFS_DMACTL1, 0);
  m_regs.addRegister(OFS_DMACTL2, 0);
  m_regs.addRegister(OFS_DMACTL4, 0);
  m_regs.addRegister(OFS_DMAIV, 0, RegisterFile::READ);
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
  if (nChannels > 6) {
    m_regs.addRegister(OFS_DMACTL3, 0);
    m_regs.addRegister(OFS_DMA6CTL, 0);
    m_regs.addRegister(OFS_DMA6SA, undef);
    m_regs.addRegister(OFS_DMA6DA, undef);
    m_regs.addRegister(OFS_DMA6SZ, undef);
    m_regs.addRegister(OFS_DMA7CTL, 0);
    m_regs.addRegister(OFS_DMA7SA, undef);
    m_regs.addRegister(OFS_DMA7DA, undef);
    m_regs.addRegister(OFS_DMA7SZ, undef);
  }

  // Set up methods & threads
  SC_METHOD(reset);
  sensitive << m_pwrOn;

  SC_THREAD(process);
}

GenericDma::reset() {
  // reset register values to defaults, or 0xAAAA if undefined
  const unsigned undef = 0xAAAA;
  m_regs(OFS_DMACTL0).write(0, /*force=*/true);
  m_regs(OFS_DMACTL1).write(0, /*force=*/true);
  m_regs(OFS_DMACTL2).write(0, /*force=*/true);
  m_regs(OFS_DMACTL3).write(0, /*force=*/true);
  m_regs(OFS_DMAIV).write(0, /*force=*/true);
  m_regs.(OFS_DMA0CTL).write(0, /*force=*/true);
  m_regs.(OFS_DMA0SA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA0DA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA0SZ).write(undef, /*force=*/true);
  m_regs.(OFS_DMA1CTL).write(0, /*force=*/true);
  m_regs.(OFS_DMA1SA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA1DA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA1SZ).write(undef, /*force=*/true);
  m_regs.(OFS_DMA2CTL).write(0, /*force=*/true);
  m_regs.(OFS_DMA2SA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA2DA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA2SZ).write(undef, /*force=*/true);
  m_regs.(OFS_DMA3CTL).write(0, /*force=*/true);
  m_regs.(OFS_DMA3SA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA3DA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA3SZ).write(undef, /*force=*/true);
  m_regs.(OFS_DMA4CTL).write(0, /*force=*/true);
  m_regs.(OFS_DMA4SA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA4DA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA4SZ).write(undef, /*force=*/true);
  m_regs.(OFS_DMA5CTL).write(0, /*force=*/true);
  m_regs.(OFS_DMA5SA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA5DA).write(undef, /*force=*/true);
  m_regs.(OFS_DMA5SZ).write(undef, /*force=*/true);
  if (nChannels > 6) {
    m_regs(OFS_DMACTL4).write(0, /*force=*/true);
    m_regs.(OFS_DMA6CTL).write(0, /*force=*/true);
    m_regs.(OFS_DMA6SA).write(undef, /*force=*/true);
    m_regs.(OFS_DMA6DA).write(undef, /*force=*/true);
    m_regs.(OFS_DMA6SZ).write(undef, /*force=*/true);
    m_regs.(OFS_DMA7CTL).write(0, /*force=*/true);
    m_regs.(OFS_DMA7SA).write(undef, /*force=*/true);
    m_regs.(OFS_DMA7DA).write(undef, /*force=*/true);
    m_regs.(OFS_DMA7SZ).write(undef, /*force=*/true);
  }

  // Update channels
}
void GenericDma::b_transport(tlm::tlm_generic_payload &trans,
                             sc_core::sc_time &delay) {
  BusTarget::b_transport(trans, delay);
  auto addr = trans.get_address();
  auto val = m_regs.read(addr);

  // Utility for setting the trigger source for a channel
  auto setTrigger = [trigger](DmaChannel &ch, int n) {
    if (n < 30) {
      ch.trigger = &trigger[n];
    } else if (n == 30) {
      spdlog::warn("Internal DMA triggers not yet implemented.");
      sc_abort();
    } else {
      spdlog::error("DMAE0 trigger not yet implemented");
      sc_abort();
    }
  };

  if (trans.get_command == TLM_WRITE_COMMAND) {
    switch (addr) {
      case OFS_DMACTL0:
        setTrigger(m_channels[0], val & 0x1f);
        setTrigger(m_channels[1], (val & (0x1f << 8)) >> 8);
        m_updateEvent.notify(delay);
        break;
      case OFS_DMACTL1:
        setTrigger(m_channels[2], val & 0x1f);
        setTrigger(m_channels[3], (val & (0x1f << 8)) >> 8);
        m_updateEvent.notify(delay);
        break;
      case OFS_DMACTL2:
        setTrigger(m_channels[4], val & 0x1f);
        setTrigger(m_channels[5], (val & (0x1f << 8)) >> 8);
        m_updateEvent.notify(delay);
        break;
      case OFS_DMACTL4:
        spdlog::warn("{}: DMACTL4 functionality is not implemented");
        break;
      case OFS_DMA0CTL:
        m_channels[0].updateChannelConfig(val);
        break;
      case OFS_DMA1CTL:
        m_channels[1].updateChannelConfig(val);
        if (val & DMAREQ) {
          m_updateEvent.notify(delay);
        }
        break;
      case OFS_DMA2CTL:
        m_channels[2].updateChannelConfig(val);
        if (val & DMAREQ) {
          m_updateEvent.notify(delay);
        }
        break;
      case OFS_DMA3CTL:
        m_channels[3].updateChannelConfig(val);
        if (val & DMAREQ) {
          m_updateEvent.notify(delay);
        }
        break;
      case OFS_DMA4CTL:
        m_channels[4].updateChannelConfig(val);
        if (val & DMAREQ) {
          m_updateEvent.notify(delay);
        }
        break;
      case OFS_DMA5CTL:
        m_channels[5].updateChannelConfig(val);
        if (val & DMAREQ) {
          m_updateEvent.notify(delay);
        }
        break;
      case (OFS_DMAIV):
        // Clear pending and set next pending
        break;
    }
  } else if (trans.get_command == TLM_READ_COMMAND) {
    switch (m_lastAccessAddress) {
      case (OFS_DMAIV):
        // Clear pending and set next pending
        break;
    }
  }
}

void GenericDma::process() {
  // Make this sensitive to triggers & updateSensitivityEvent
  wait(SC_ZERO_TIME);  // Wait for simulation start

  // Wait for any channel to be enabled
  while (m_channels.end() ==
         std::find_if(m_channels.begin(), m_channels.end(),
                      [](const auto &c) { return c.enable; })) {
    wait(m_writeEvent);
  }

  while (true) {
    // Wait for any trigger
    sc_event triggers;
    for (const auto &c : m_channels) {
      triggers |= *c.trigger.posedge_event();
    }
    wait(triggers);

    // Find highest-priority (lowest channel number) triggered channel
    auto it = m_channels.end();
    while (it == m_channels.end()) {
      std::find_if(m_channels.begin(), m_channels.end(),
                   [](const auto &c) { return c.trigger->read(); });
    }
    auto &ch = *it;
    const auto channelIdx = it - m_channels.begin();

    // DMA spends "1 or 2 clock cycles to synchronize to mclk"
    wait(clk->getPeriod());

    // Utility to check for breakout condition during block/block-burst
    // transfers
    auto breakout = [ch]() {
      return !ch.enable && !(ch.level & ch.trigger.read());
    };

    // Utility to update size register (DMAxSZ)
    auto decrementSize = [channelIdx, m_regs](const int n) {};

    switch (ch.transferMode) {
      case DmaChannel::TransferMode::Single:
        // One transfer per trigger
        transfer(ch);
        break;
      case DmaChannel::TransferMode::Block:
        // Transfer whole block after first trigger
        for (auto i = 0; i < ch.size; i++) {
          transfer(ch);
          if (breakout) {
            break;
          }
        }
        ch.enable = false;
        break;
      case DmaChannel::TransferMode::BurstBlock:
        //  CPU activity interleaved with transfer
        //  Delay for 2 cycles every 4 transfers
        for (auto i = 0; i < ch.size; i++) {
          transfer(ch);
          if ((i > 0) && (i % 4 == 0)) {
            wait(2 * clk->getPeriod());
          }
          if (breakout) {
            break;
          }
        }
        ch.enable = false;
        break;
      case DmaChannel::TransferMode::RepeatedBlock:
        // Transfer one whole block after first trigger, don't disable enable
        for (auto i = 0; i < ch.size; i++) {
          transfer(ch);
          if (breakout) {
            break;
          }
        }
        break;
      case DmaChannel::TransferMode::RepeatedBurstBlock:
        // Continue burst-transfer indefinitely after first trigger, until
        // breakout condition
        while (1) {
          for (auto i = 0; i < ch.size; i++) {
            transfer(ch);
            if ((i > 0) && (i % 4 == 0)) {
              wait(2 * clk->getPeriod());
            }
            if (breakout) {
              break;
            }
          }
        }
        break;
    }
  }
}

GenericDma::transfer(int channelNumber) {
  uint8_t data[2];
  sc_time delay = SC_ZERO_TIME;
  tlm_generic_payload trans;  //! Outgoing transaction
  trans.set_data_ptr(data);

  auto &ch = m_channels(channelNumber);

  // Read
  trans.set_command(TLM_READ_COMMAND);
  trans.set_address(ch.sourceAddress);
  trans.set_data_length((ch.sourceBytes == DmaChannel::Bytes::Byte) ? 1 : 2);
  delay = SC_ZERO_TIME;
  iSocket->b_transport(trans, delay);
  wait(delay);

  // Write
  trans.set_address(ch.destinationAddress);
  trans.set_command(TLM_WRITE_COMMAND);
  delay = SC_ZERO_TIME;
  iSocket->b_transport(trans, delay);
  wait(delay);

  // Update channel state
  ch.updateAddresses();
  auto offset = channelNumber * (OFS_DMA1CTL - OFS_DMA0CTL);
  auto tmp = m_regs.read(OFS_DMA0SZ + offset);
  if (tmp > 0) {
    m_regs.write(OFS_DMA0SZ + offset, tmp - 1);
  } else {
    // Job done, reset size, destination and source
    m_regs.write(OFS_DMA0SZ + offset, ch.size);
    ch.sourceAddress = m_regs.read(OFS_DMA0SA + offset);
    ch.destinationAddress = m_regs.read(OFS_DMA0DA + offset);
    ch.enable &=  // Clear enable unless repeated transfer mode
        (ch.TransferMode == DmaChannel::TransferMode::RepeatedSingle) |
        (ch.TransferMode == DmaChannel::TransferMode::RepeatedBlock) |
        (ch.TransferMode == DmaChannel::TransferMode::RepeatedBurstBlock);
  }
}

void DmaChannel::updateAddresses() {
  switch (destinationAutoIncrement) {
    case AutoIncrementMode::Unchanged:
      break;
    case AutoIncrementMode::Decrement:
      m_tDestinationAddress += (destinationBytes == Bytes::Word) ? 2 : 1;
      break;
    case AutoIncrementMode::Increment:
      m_tDestinationAddress -= (destinationBytes == Bytes::Word) ? 2 : 1;
      break;
  }
  switch (sourceAutoIncrement) {
    case AutoIncrementMode::Unchanged:
      break;
    case AutoIncrementMode::Decrement:
      m_tSourceAddress += (sourceBytes == Bytes::Word) ? 2 : 1;
      break;
    case AutoIncrementMode::Increment:
      m_tSourceAddress -= (sourceBytes == Bytes::Word) ? 2 : 1;
      break;
  }
}

DmaChannel::updateConfig(const unsigned cfg) {
  softwareTrigger = cfg & DMAREQ;
  interruptEnable = cfg & DMAIE;
  enable = cfg & DMAEN;
  levelSensitive = cfg & DMALEVEL;
  sourceBytes = (cfg & DMASRCBYTE) ? Bytes::Byte : Bytes::Word;
  destinationBytes = (cfg & DMADSTBYTE) ? Bytes::Byte : Bytes::Word;

  switch ((cfg & DMASRCINCR) >> 8) {
    case 0:
      sourceAutoIncrement = AutoIncrementMode::Unchanged;
    case 1:
      sourceAutoIncrement = AutoIncrementMode::Unchanged;
    case 2:
      sourceAutoIncrement = AutoIncrementMode::Decrement;
    case 3:
      sourceAutoIncrement = AutoIncrementMode::Increment;
  }

  switch ((cfg & DMADSTINCR) >> 10) {
    case 0:
      sourceAutoIncrement = AutoIncrementMode::Unchanged;
    case 1:
      sourceAutoIncrement = AutoIncrementMode::Unchanged;
    case 2:
      sourceAutoIncrement = AutoIncrementMode::Decrement;
    case 3:
      sourceAutoIncrement = AutoIncrementMode::Increment;
  }

  switch ((cfg & DMADT) >> 12) {
    case 0:
      transferMode = TransferMode::Single;
    case 1:
      transferMode = TransferMode::Block;
    case 2:
      transferMode = TransferMode::BurstBlock;
    case 3:
      transferMode = TransferMode::BurstBlock;
    case 4:
      transferMode = TransferMode::RepeatedSingle;
    case 5:
      transferMode = TransferMode::RepeatedBlock;
    case 6:
      transferMode = TransferMode::RepeatedBurstBlock;
    case 7:
      transferMode = TransferMode::RepeatedBurstBlock;
  }

  // Software trigger
  if (cfg & DMAREQ) {
    m_softwareTrigger.notify(SC_ZERO_TIME);
  }
}

void DmaChannel::setTrigger(const sc_event *e) { m_trigger = e; }

void DmaChannel::end_of_elaboration() {
  SC_THREAD(process);
  sensitive << *m_trigger << m_softwareTrigger;
}

void DmaChannel::process() {
  wait(SC_ZERO_TIME);  // Wait for simulation to start

  while (true) {
    while (!enable) {
      wait(*m_trigger | softwareTrigger);
    }

    // DMA spends "1 or 2 clock cycles to synchronize to mclk"
    wait(clk->getPeriod());

    // Transfer sequence
    auto transfer = [=]() {
      pending.write(true);
      wait(accept.posedge_event());
      updateState();
      pending.write(false); wait(accept.negedge_event();
    };

    switch (transferMode) {
      case TransferMode::Single:
        // One transfer per trigger
        transfer();
        break;
      case DmaChannel::TransferMode::Block:
        // Transfer whole block after first trigger
        for (auto i = 0; i < m_size; i++) {
          transfer();
          if (!enable) {
            break;
          }
        }
        ch.enable = false;
        break;
      case DmaChannel::TransferMode::BurstBlock:
        //  CPU activity interleaved with transfer
        //  Delay for 2 cycles every 4 transfers
        for (auto i = 0; i < m_size; i++) {
          transfer();
          if ((i > 0) && (i % 4 == 0)) {
            wait(2 * clk->getPeriod());
          }
          if (!enable) {
            break;
          }
        }
        ch.enable = false;
        break;
      case DmaChannel::TransferMode::RepeatedBlock:
        // Transfer one whole block after first trigger, don't clear enable
        for (auto i = 0; i < m_size; i++) {
          transfer();
          if (!enable) {
            break;
          }
        }
        break;
      case DmaChannel::TransferMode::RepeatedBurstBlock:
        // Continue burst-transfer indefinitely after first trigger, until
        // breakout condition
        while (1) {
          for (auto i = 0; i < m_size; i++) {
            transfer();
            if ((i > 0) && (i % 4 == 0)) {
              wait(2 * clk->getPeriod());
            }
            if (breakout) {
              break;
            }
          }
        }
        break;
    }
  }
}

void DmaChannel::updateState() {
  updateAddresses();
  if (size > 0) {
    size--;
  } else {
    // Job done, reset size, destination and source
    m_tSize = size;
    m_tSourceAddress = sourceAddress;
    m_tDestinationAddress = destinationAddress;
    enable &=  // Clear enable unless repeated transfer mode
        (transferMode == TransferMode::RepeatedSingle) |
        (transferMode == TransferMode::RepeatedBlock) |
        (transferMode == TransferMode::RepeatedBurstBlock);
  }
}
