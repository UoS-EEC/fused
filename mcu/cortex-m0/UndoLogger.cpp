/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#include "libs/make_unique.hpp"
#include "mcu/cortex-m0/UndoLogger.hpp"
#include "ps/ConstantCurrentState.hpp"
#include "ps/ConstantEnergyEvent.hpp"
#include "utilities//Utilities.hpp"
#include <iostream>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>

using namespace sc_core;
using namespace tlm;

UndoLogger::UndoLogger(const sc_module_name name,
                       const unsigned memStartAddress,
                       const unsigned ctrlStartAddress, const int capacity,
                       const int cacheLineWidth, const int exceptionId)
    : BusTarget(name, ctrlStartAddress,
                ctrlStartAddress + RegisterAddress::FLUSH_SIZE + 4 - 1),
      m_memStartAddress(memStartAddress), m_cacheLineWidth(cacheLineWidth),
      m_capacity(capacity), m_exceptionId(exceptionId) {
  // Build register file
  m_regs.addRegister(RegisterAddress::CTRL, BitMasks::CTRL_FLUSHEN);
  m_regs.addRegister(RegisterAddress::STATUS,
                     /*resetValue=*/
                     BitMasks::STATUS_EMPTY_MASK |
                         ((capacity << BitMasks::STATUS_CAPACITY_SHIFT) &
                          BitMasks::STATUS_CAPACITY_MASK),
                     RegisterFile::AccessMode::READ);
  m_regs.addRegister(RegisterAddress::FIFO);
  m_regs.addRegister(RegisterAddress::FIFO_THR, /*resetValue=*/capacity);
  m_regs.addRegister(RegisterAddress::UNSAFE_BASE, /*resetValue=*/0x0);
  m_regs.addRegister(RegisterAddress::UNSAFE_SIZE);
  m_regs.addRegister(RegisterAddress::FLUSH_BASE);
  m_regs.addRegister(RegisterAddress::FLUSH_SIZE);

  // Register handlers for initiator socket
  memSocket.register_b_transport(this, &UndoLogger::mem_b_transport);
  memSocket.register_transport_dbg(this, &UndoLogger::mem_transport_dbg);
}

void UndoLogger::end_of_elaboration() {
  BusTarget::end_of_elaboration();

  // Register power modelling events & states
  m_readLogByteEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "read log byte"));
  m_writeLogByteEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "write log byte"));

  m_offStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "off"));
  m_disabledStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "disabled"));
  m_enabledStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "enabled"));

  // Set up methods
  SC_HAS_PROCESS(UndoLogger);
  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_METHOD(irqControl);
  sensitive << returning_exception << m_updateIrqEvent;
  dont_initialize();

  SC_THREAD(dmaTriggerControl);

  SC_THREAD(apply);

  SC_THREAD(flush);
}

void UndoLogger::reset() {
  // Issue some useful warnings
  if (size() > 0) {
    SC_REPORT_WARNING(
        this->name(),
        fmt::format(FMT_STRING("reset while log contains {:d} entries, will "
                               "probably result in corrupt state!"),
                    size())
            .c_str());
  }
  if (m_applying) {
    SC_REPORT_WARNING(this->name(), "reset while applying log!");
  }

  // Reset internal variables
  m_regs.reset();
  m_setIrq = false;
  m_enable = false;
  m_overflow = false;
  m_applying = false;
  m_nFlushedEntries = 0;
  m_log.clear();

  if (pwrOn.read() == false) {
    powerModelPort->reportState(m_offStateId);
  }
}

void UndoLogger::b_transport(tlm::tlm_generic_payload &trans,
                             sc_core::sc_time &delay) {
  BusTarget::b_transport(trans, delay);

  const auto addr = trans.get_address();
  const auto val = m_regs.read(addr);
  const bool isWrite = trans.get_command() == tlm::TLM_WRITE_COMMAND;

  if (trans.get_data_length() != 4 || addr % 4 != 0) {
    SC_REPORT_FATAL(
        this->name(),
        fmt::format(
            FMT_STRING("invalid access (addr={:d}, len={:d}). UndoLogger "
                       "accepts aligned word (32-bit) accesses only."),
            addr, trans.get_data_length())
            .c_str());
  }

  // Handle special cases
  switch (addr) {
  case RegisterAddress::CTRL:
    if (val & BitMasks::CTRL_ENABLE && !m_enable) {
      powerModelPort->reportState(m_enabledStateId);
      m_enable = true;
    } else if (!(val & BitMasks::CTRL_ENABLE) && m_enable) {
      powerModelPort->reportState(m_disabledStateId);
      m_enable = false;
    }

    if (val & BitMasks::CTRL_CLEAR) {
      if (!m_applying) {
        powerModelPort->reportEvent(m_writeLogByteEventId,
                                    m_log.size() * (4 + m_cacheLineWidth));
        m_log.clear();
        m_nFlushedEntries = 0;
      } else {
        SC_REPORT_WARNING(this->name(),
                          "Attempt to clear log while applying ignored.");
      }
    }
    break;
  case RegisterAddress::FIFO:
    if (m_enable) {
      SC_REPORT_WARNING(
          this->name(),
          "Accessing FIFO while logging enabled can corrupt the log");
    }
    if (isWrite) {
      pushLogWord(val);
    } else {
      // Read from log, not from register
      m_regs.write(RegisterAddress::FIFO, popLogWord());

      // Clear overflow when log is empty
      m_overflow &= size() > 0;

      // Overwrite response data
      m_regs.read(RegisterAddress::FIFO, trans.get_data_ptr(), 4);
    }
    break;
  }

  // Update state, in case it changed
  updateStatusRegister();
}

void UndoLogger::mem_b_transport(tlm::tlm_generic_payload &trans,
                                 sc_core::sc_time &delay) {
  // Incoming transactions should be cache line width wide and aligned
  sc_assert(trans.get_data_length() == m_cacheLineWidth);
  sc_assert(trans.get_address() % trans.get_data_length() == 0);
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND && m_enable &&
      filter(trans.get_address())) {
    logLine(trans.get_address(), delay);
  }

  // Forward transaction to downstream memory
  iSocket->b_transport(trans, delay);
  return;
}

unsigned int UndoLogger::mem_transport_dbg(tlm::tlm_generic_payload &trans) {
  // Forward transaction to downstream memory
  return iSocket->transport_dbg(trans);
}

int UndoLogger::size() const {
  // Each entry contains a 4-byte address plus data
  const auto res = m_log.size() / (m_cacheLineWidth + 4);
  return res;
}

void UndoLogger::logLine(const unsigned address, sc_core::sc_time &delay) {
  if (m_capacity - size() <= 0) {
    // Log will oveflow, set flag and remove an entry from front
    m_overflow = true;
    for (int i = 0; i < 4 + m_cacheLineWidth; ++i) {
      m_log.pop_front();
    }
  }

  // Make sure address is aligned
  sc_assert(address % m_cacheLineWidth == 0);

  // Read downstream value
  std::vector<uint8_t> data(m_cacheLineWidth, 0);
  tlm::tlm_generic_payload trans;
  trans.set_address(address);
  trans.set_data_length(data.size());
  trans.set_data_ptr(&data[0]);
  trans.set_command(tlm::TLM_READ_COMMAND);
  iSocket->b_transport(trans, delay);

  // Write to log
  // [address, [data[0], data[1], ...]]

  // Store absolute address
  pushLogWord(trans.get_address() + m_memStartAddress);

  // Store data
  powerModelPort->reportEvent(m_writeLogByteEventId, data.size());
  for (const auto &d : data) {
    m_log.push_back(d);
  }

  spdlog::info(FMT_STRING("{:s}:logLine @{:s} addr=0x{:08x}. Log now contains "
                          "{:d}/{:d} entries."),
               this->name(), sc_time_stamp().to_string(),
               address + m_memStartAddress, size(), m_capacity);

  m_logEvent.notify(SC_ZERO_TIME);
  updateStatusRegister();
}

void UndoLogger::flush() {
  auto data = std::vector<uint8_t>(m_cacheLineWidth, 0);
  tlm::tlm_generic_payload trans;
  trans.set_command(tlm::TLM_WRITE_COMMAND);
  sc_time delay;

  wait(SC_ZERO_TIME);

  while (true) {
    wait(m_writeEvent | m_logEvent);
    if ((m_regs.read(RegisterAddress::CTRL) & BitMasks::CTRL_FLUSHEN) &&
        size() >= m_regs.read(RegisterAddress::FIFO_THR) && size() > 0) {
      const unsigned blockEntries = m_cacheLineWidth / 4; // entries in  a block
      const unsigned blockSize =
          (blockEntries + 1) * m_cacheLineWidth; // Size of one block
      const unsigned block = m_nFlushedEntries / blockEntries;
      const unsigned offset = m_nFlushedEntries % blockEntries;
      const unsigned addressAddress = m_regs.read(RegisterAddress::FLUSH_BASE) -
                                      m_memStartAddress + block * blockSize +
                                      offset * 4;
      const unsigned dataAddress = m_regs.read(RegisterAddress::FLUSH_BASE) -
                                   m_memStartAddress + block * blockSize +
                                   (1 + offset) * m_cacheLineWidth;

      // Get absolute address
      auto entryAddr = popLogWord();

      // Copy data to transaction
      std::copy(m_log.begin(), m_log.begin() + m_cacheLineWidth, data.begin());
      for (int i = 0; i < m_cacheLineWidth; ++i) {
        m_log.pop_front();
      }

      // Write data
      delay = SC_ZERO_TIME;
      trans.set_data_length(m_cacheLineWidth);
      trans.set_address(dataAddress);
      trans.set_data_ptr(&data[0]);
      iSocket->b_transport(trans, delay);

      // Write address
      trans.set_data_length(4);
      Utility::unpackBytes(data.data(), Utility::htotl(entryAddr), 4);
      trans.set_address(addressAddress);
      iSocket->b_transport(trans, delay);
      wait(delay);

      m_nFlushedEntries++;

      spdlog::info(
          FMT_STRING("{:s}:flush @flushed entry with address 0x{:08x}"),
          this->name(), entryAddr);

      if (dataAddress + m_cacheLineWidth -
              (m_regs.read(RegisterAddress::FLUSH_BASE) - m_memStartAddress) >
          blockSize * m_regs.read(RegisterAddress::FLUSH_SIZE)) {
        SC_REPORT_FATAL(this->name(), "Flush address outside bounds.");
      }

      powerModelPort->reportEvent(m_readLogByteEventId, m_cacheLineWidth + 4);
      updateStatusRegister();
    }
  }
}

void UndoLogger::apply() {
  // Set up transaction
  auto data = std::vector<uint8_t>(m_cacheLineWidth, 0);
  tlm::tlm_generic_payload trans;
  trans.set_data_length(m_cacheLineWidth);
  trans.set_command(tlm::TLM_WRITE_COMMAND);
  sc_time delay;

  wait(SC_ZERO_TIME);

  while (true) {
    wait(m_writeEvent);
    delay = SC_ZERO_TIME;
    if (m_regs.testBitMask(RegisterAddress::CTRL, BitMasks::CTRL_APPLY)) {
      spdlog::info(FMT_STRING("{:s} applying ~{:d} entries"), this->name(),
                   size());
      m_applying = true;

      // Write entries
      // This should tolerate the case where software adds more entries to the
      // log during the apply, so we pop one and one value off m_log, and
      // continue until there are no more complete entries.
      while (size()) {
        // Get relative address
        auto addr = popLogWord() - m_memStartAddress;

        // Sanity check: make sure address is aligned
        sc_assert(addr % m_cacheLineWidth == 0);

        // Write entry to downstream memory

        // Prepare a contiguous vector for the copy (m_log is not contiguous)
        std::vector<uint8_t> data(m_cacheLineWidth, 0);
        std::copy(m_log.begin(), m_log.begin() + m_cacheLineWidth,
                  data.begin());

        trans.set_address(addr);
        trans.set_data_ptr(&data[0]);
        iSocket->b_transport(trans, delay);

        powerModelPort->reportEvent(m_readLogByteEventId, m_cacheLineWidth);
        for (int i = 0; i < m_cacheLineWidth; ++i) {
          m_log.pop_front();
        }
        wait(delay);
        updateStatusRegister();
      }

      // Cleanup
      m_overflow = false;
      m_applying = false;
      m_regs.clearBitMask(RegisterAddress::CTRL, BitMasks::CTRL_APPLY);
      updateStatusRegister();
    }
  }
}

bool UndoLogger::filter(unsigned address) const {
  // Subtract the attached memory's base address from the UNSAFE address
  const auto start =
      m_regs.read(RegisterAddress::UNSAFE_BASE) - m_memStartAddress;
  const auto end = start + m_regs.read(RegisterAddress::UNSAFE_SIZE);
  return !(address >= start && address < end);
}

void UndoLogger::pushLogWord(const unsigned val) {
  uint8_t data[4];
  Utility::unpackBytes(data, Utility::htotl(val), 4);
  powerModelPort->reportEvent(m_writeLogByteEventId, 4);
  for (int i = 0; i < 4; ++i) {
    m_log.push_back(data[i]);
  }
}

unsigned UndoLogger::popLogWord() {
  uint8_t data[4];
  powerModelPort->reportEvent(m_readLogByteEventId, 4);
  if (m_log.size() < 4) {
    SC_REPORT_FATAL(this->name(), "Attempt to pop empty log.");
  }
  for (int i = 0; i < 4; ++i) {
    data[i] = m_log.front();
    m_log.pop_front();
  }
  return Utility::ttohl(Utility::packBytes(data, 4));
}

void UndoLogger::updateStatusRegister() {
  m_regs.write(RegisterAddress::STATUS,
               ((size() == 0) << BitMasks::STATUS_EMPTY_SHIFT) |     // Empty
                   (m_overflow << BitMasks::STATUS_OVERFLOW_SHIFT) | // Overflow
                   ((size() >= m_regs.read(RegisterAddress::FIFO_THR))
                    << BitMasks::STATUS_THRESHOLD_SHIFT) | // Threshold
                   ((m_capacity - size())
                    << BitMasks::STATUS_FREESLOTS_SHIFT) | // Free slots
                   ((m_capacity << BitMasks::STATUS_CAPACITY_SHIFT) &
                    BitMasks::STATUS_CAPACITY_MASK), // Capacity
               /*force=*/true);

  // Update IRQ
  if (m_regs.testBitMask(RegisterAddress::CTRL, BitMasks::CTRL_IE) &&
      m_regs.testBitMask(RegisterAddress::STATUS,
                         BitMasks::STATUS_THRESHOLD_MASK)) {
    m_setIrq = true;
    m_updateIrqEvent.notify(SC_ZERO_TIME);
  }

  // Update dmaTrigger
  if (m_regs.testBitMask(RegisterAddress::CTRL, BitMasks::CTRL_DMAEN) &&
      m_regs.testBitMask(RegisterAddress::STATUS,
                         BitMasks::STATUS_THRESHOLD_MASK)) {
    m_setDmaTrigger = true;
    m_updateDmaTriggerEvent.notify(SC_ZERO_TIME);
  }
}

std::ostream &operator<<(std::ostream &os, const UndoLogger &rhs) {
  // Print
  // clang-format off
  os << "<UndoLogger> " << rhs.name()
    << fmt::format(FMT_STRING("\nCacheLineWidth={:d}, Capacity={:d}"),
        rhs.m_cacheLineWidth, rhs.m_capacity)
    << "\nenable: " << rhs.m_enable
    << "\nsetIrq: " << rhs.m_setIrq
    << "\noverflow: " << rhs.m_overflow
    << "\napplying: " << rhs.m_applying
    << "\nRegisters: " << rhs.m_regs
    << "\nFree slots: " << rhs.m_capacity - rhs.size() << " entries"
    << "\nLog size: " << rhs.size() << " entries"
    << "\nLog (complete entries): ";
  // clang-format on

  // Complete log entries
  for (int i = 0; i < rhs.size() * (rhs.m_cacheLineWidth + 4);) {
    uint8_t data[4];
    for (int j = 0; j < 4; ++j) {
      data[j] = rhs.m_log[i];
      i += 1;
    }
    os << fmt::format(FMT_STRING("\n\tAddress = 0x{:08x} data=["),
                      Utility::ttohl(Utility::packBytes(data, 4)));
    for (int j = 0; j < rhs.m_cacheLineWidth; ++j) {
      os << fmt::format(FMT_STRING("0x{:02x}"), rhs.m_log[i])
         << ((j < rhs.m_cacheLineWidth - 1) ? ", " : "]");
      i += 1;
    }
  }

  // Report incomplete log entries
  const auto extraBytes =
      rhs.m_log.size() - rhs.size() * (rhs.m_cacheLineWidth + 4);
  if (extraBytes > 0) {
    os << "\nIncomplete data (partial entry): {";
    for (auto it = rhs.m_log.cend() - extraBytes; it != rhs.m_log.cend();
         ++it) {
      os << fmt::format(FMT_STRING("0x{:02x}"), *it);
      os << ((it < rhs.m_log.end() - 1) ? ", " : "}");
    }
  }

  // clang-format on
  return os;
}

void UndoLogger::irqControl() {
  if (pwrOn.read() == false) {
    irq.write(false);
  } else if (m_setIrq && (!irq.read())) {
    spdlog::info("{:s}: @{:s} interrupt request", this->name(),
                 sc_time_stamp().to_string());
    irq.write(true);
  } else if (returning_exception.read() - 16 == m_exceptionId) {
    spdlog::info("{:s}: @{:s} interrupt request cleared.", this->name(),
                 sc_time_stamp().to_string());
    irq.write(false);
  }
  m_setIrq = false;
}

void UndoLogger::dmaTriggerControl() {
  dmaTrigger.write(false);
  while (true) {
    wait(pwrOn.default_event() | m_updateDmaTriggerEvent);
    if (pwrOn.read() == false) {
      dmaTrigger.write(false);
    } else if (m_setDmaTrigger && !dmaTrigger.read()) {
      dmaTrigger.write(true);
      spdlog::info("{}: @{:s} DMA trigger", this->name(),
                   sc_time_stamp().to_string());
    }
    m_setDmaTrigger = false;
    wait(systemClk->getPeriod());
    dmaTrigger.write(false);
  }
}
