/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "libs/make_unique.hpp"
#include "mcu/Cache.hpp"
#include "mcu/CacheController.hpp"
#include "mcu/CacheReplacementPolicies.hpp"
#include "ps/ConstantCurrentState.hpp"
#include "ps/ConstantEnergyEvent.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"
#include <cmath>
#include <iostream>
#include <random>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>

using namespace sc_core;

CacheSet::CacheSet(const int nLines, const int lineWidth,
                   CacheReplacementIf *replacementPolicy)
    : replacementPolicy(replacementPolicy), lines(nLines, lineWidth) {}

int CacheSet::findLine(const unsigned tag) {
  for (unsigned int i = 0; i < lines.size(); i++) {
    if (lines[i].tag == tag && lines[i].valid) {
      return i; // Cache hit
    }
  }
  return -1; // Cache miss
}

void CacheSet::reset() {
  for (auto &l : lines) {
    l.reset();
  }
  replacementPolicy->reset();
}

Cache::Cache(const sc_module_name name, const unsigned startAddress,
             const unsigned endAddress, const unsigned ctrlAddress)
    : BusTarget(name, startAddress, endAddress),
      cacheCtrl((std::string(name) + "_ctrl").c_str(), ctrlAddress) {
  iSocket.bind(*this);

  // Bind submodules
  cacheCtrl.nDirtyLines.bind(nDirtyLines);
  cacheCtrl.doFlush.bind(doFlush);

  // Config
  std::string strname = this->name();
  auto cfg = m_lineWidth = Config::get().getUint(strname + ".CacheLineWidth");
  m_nSets = Config::get().getUint(strname + ".CacheNSets");
  m_nLines = Config::get().getUint(strname + ".CacheNLines");
  m_nOffsetBits = static_cast<int>(log2(m_lineWidth));
  m_offsetMask = m_lineWidth - 1;
  m_nIdBits = static_cast<int>(log2(m_nSets));
  m_nTagBits = static_cast<int>(log2(endAddress - startAddress + 1)) -
               m_nIdBits - m_nOffsetBits;
  m_idMask = (m_nSets - 1) << m_nOffsetBits;
  m_tagMask = ~(m_offsetMask | m_idMask);

  auto wp = Config::get().getString(strname + ".CacheWritePolicy");
  if (wp == "WriteThrough") {
    m_writePolicy = WP_WRITE_THROUGH;
  } else if (wp == "WriteAround") {
    m_writePolicy = WP_WRITE_AROUND;
  } else if (wp == "WriteBack") {
    m_writePolicy = WP_WRITE_BACK;
  }

  // Construct sets
  auto setCfg = Config::get().getString(std::string(this->name()) +
                                        ".CacheReplacementPolicy");

  for (unsigned int i = 0; i < m_nSets; i++) {
    if (setCfg == "LRU") {
      m_sets.push_back(
          CacheSet(m_nLines, m_lineWidth, new CacheReplacementLru(m_nLines)));
    } else if (setCfg == "LRUNotDirty") {
      m_sets.push_back(CacheSet(m_nLines, m_lineWidth,
                                new CacheReplacementLruNotDirty(m_nLines)));
    } else if (setCfg == "LFU") {
      m_sets.push_back(CacheSet(m_nLines, m_lineWidth,
                                new CacheReplacementLfu(m_nLines, 64)));
    } else if (setCfg == "RoundRobin") {
      m_sets.push_back(CacheSet(m_nLines, m_lineWidth,
                                new CacheReplacementRoundRobin(m_nLines)));
    } else if (setCfg == "RoundRobinNotDirty") {
      m_sets.push_back(
          CacheSet(m_nLines, m_lineWidth,
                   new CacheReplacementRoundRobinNotDirty(m_nLines)));
    } else if (setCfg == "PseudoRandom") {
      m_sets.push_back(CacheSet(m_nLines, m_lineWidth,
                                new CacheReplacementPseudoRandom(m_nLines)));
    } else if (setCfg == "PseudoRandomNotDirty") {
      m_sets.push_back(
          CacheSet(m_nLines, m_lineWidth,
                   new CacheReplacementPseudoRandomNotDirty(m_nLines)));
    } else {
      spdlog::error("FRAM Cache set: invalid replacement policy {}", setCfg);
      SC_REPORT_ERROR("FRAM Cache set", "Invalid replacement policy.");
    }
  }
};

void Cache::end_of_elaboration() {
  BusTarget::end_of_elaboration();

  // Register events
  m_readMissEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "read miss"));
  m_readHitEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "read hit"));
  m_writeMissEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "write miss"));
  m_writeHitEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "write hit"));
  m_nBytesReadEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "bytes read"));
  m_nBytesWrittenEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "bytes written"));
  m_tagReadBitEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "tag read bits"));
  m_tagWriteBitEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(this->name(), "tag write bits"));
  m_offStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(this->name(), "off"));

  const auto memoryBits =
      8 * m_lineWidth * m_nLines * m_nSets + m_nLines * m_nSets * m_nTagBits;
  const auto leakage =
      Config::get().getDouble(std::string(this->name()) + " on") * memoryBits;

  m_onStateId = powerModelPort->registerState(
      this->name(), std::make_unique<ConstantCurrentState>("on", leakage));

  // Methods & threads
  SC_METHOD(reset);
  sensitive << pwrOn;

  // Register methods
  SC_METHOD(updatePowerState);
  sensitive << pwrOn.value_changed();

  SC_METHOD(powerOffChecks);
  sensitive << pwrOn.negedge_event();
  dont_initialize();

  SC_METHOD(updateNDirtyLines);
  sensitive << m_updateNDirtyLinesEvent;

  SC_THREAD(flush);
}

void Cache::updateNDirtyLines() {
  if (m_nDirtyLines != nDirtyLines.read()) {
    nDirtyLines.write(m_nDirtyLines);
  }
}

void Cache::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  const auto addr = trans.get_address();
  uint8_t *dataPtr = trans.get_data_ptr();
  const auto len = trans.get_data_length();

  // Check alignment to cache line
  if (offset(addr) > offset(addr + len - 1)) {
    spdlog::error(
        "Transaction with address  0x{:08x} and length {:d} unaligned to "
        "cache "
        "line.",
        addr, len);
    SC_REPORT_FATAL(this->name(), "Access unaligned to cache line.");
  }

  int lineIdx = m_sets[index(addr)].findLine(tag(addr));
  // Assumes parallel read of all tags in this set on access
  powerModelPort->reportEvent(m_tagReadBitEventId, m_nTagBits * m_nLines);

  assert(lineIdx >= 0 ? lineIdx < m_nLines : 1);
  const bool hit = lineIdx >= 0;
  if (hit) {
    m_sets[index(addr)].replacementPolicy->hit(
        lineIdx, trans.get_command() == tlm::TLM_WRITE_COMMAND);
  } else { // Miss -- pick victim line
    lineIdx = m_sets[index(addr)].replacementPolicy->miss(
        trans.get_command() == tlm::TLM_WRITE_COMMAND);
    assert(lineIdx < m_nLines);
  }

  auto &line = m_sets[index(addr)].lines[lineIdx];

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    m_writeEvent.notify(delay + systemClk->getPeriod());
    powerModelPort->reportEvent(m_writeEventId);

    if (hit) {
      powerModelPort->reportEvent(m_writeHitEventId);
    } else {
      powerModelPort->reportEvent(m_writeMissEventId);
    }

    tlm::tlm_generic_payload outputTrans;
    switch (m_writePolicy) {
    case WP_WRITE_AROUND: // Update memory only
      sc_assert(line.dirty == false);
      outputTrans.set_address(addr);
      outputTrans.set_data_length(len);
      outputTrans.set_data_ptr(dataPtr);
      outputTrans.set_command(tlm::TLM_WRITE_COMMAND);
      iSocket->b_transport(outputTrans, delay);
      if (hit) {
        line.valid = false;
      }
      break;
    case WP_WRITE_THROUGH: // Update cache line & memory
      sc_assert(line.dirty == false);
      if (!hit) {
        readLine(addr, line, delay); // load new line
      }

      // Update cached data
      memcpy(&line.data[offset(addr)], dataPtr, len);
      powerModelPort->reportEvent(m_nBytesWrittenEventId, 4 /* 32-bit word */);

      // Update memory
      outputTrans.set_address(addr);
      outputTrans.set_data_length(len);
      outputTrans.set_data_ptr(dataPtr);
      outputTrans.set_command(tlm::TLM_WRITE_COMMAND);
      iSocket->b_transport(outputTrans, delay);
      break;
    case WP_WRITE_BACK: // Update cache line only
      if (!hit && line.valid && line.dirty) {
        writeLine(line, addr, delay); // Write back victim before loading
      }

      if (!hit) {
        readLine(addr, line, delay); // load new line
      }

      // Update cached data
      memcpy(&line.data[offset(addr)], dataPtr, len);
      powerModelPort->reportEvent(m_nBytesWrittenEventId, 4 /* 32-bit word */);

      if (line.dirty == false) {
        m_nDirtyLines++;
      }
      line.dirty = true;
      break;
    default:
      SC_REPORT_FATAL(this->name(), "Invalid write policy.");
      break;
    }
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    m_readEvent.notify(delay + systemClk->getPeriod());
    powerModelPort->reportEvent(m_readEventId);

    if (hit) {
      powerModelPort->reportEvent(m_readHitEventId);
    } else {
      powerModelPort->reportEvent(m_readMissEventId);
    }

    if (!hit) {
      // Miss -- Fetch data from memory before serving
      if (line.valid && line.dirty) {
        writeLine(line, addr, delay); // Write back victim first
      }
      readLine(addr, line, delay); // Fetch new line
    }

    // Return data
    std::memcpy(dataPtr, &line.data[offset(addr)], len);
    powerModelPort->reportEvent(m_nBytesReadEventId, 4 /* 32-bit word */);
  } else {
    SC_REPORT_FATAL(this->name(), "Transaction command not supported.");
  }

  m_updateNDirtyLinesEvent.notify(SC_ZERO_TIME);

  // Sanity check

  if (m_nDirtyLines != countDirtyLines()) {
    spdlog::error("{}: invalid m_nDirtyLines {} (should be {})", this->name(),
                  m_nDirtyLines, countDirtyLines());
    std::cerr << *this;
    SC_REPORT_FATAL(this->name(), "Invalid count of dirty lines");
  }

  delay += systemClk->getPeriod();
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned int Cache::transport_dbg(tlm::tlm_generic_payload &trans) {
  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    // Write-transactions invalidate entire cache and go straight to memory
    debugFlushAndInvalidate();
    return iSocket->transport_dbg(trans);
  }

  // Read transactions are broken down into cache lines
  auto address = trans.get_address();
  int remaining = trans.get_data_length();
  auto *dataPtr = trans.get_data_ptr();

  // Break up large accesses
  while (remaining > 0) {
    // Take up to a line if address aligned, otherwise take a byte
    const int len =
        (address % m_lineWidth == 0) ? std::min(m_lineWidth, remaining) : 1;

    // Check cache first
    try {
      auto &line = findLine(address);
      memcpy(dataPtr, &line.data[offset(address)], len);
      // spdlog::info("read from cache");
    } catch (std::runtime_error &e) {
      // Value is not in cache. Read from downstream.
      // Ouput transaction object needed to break down incoming transaction
      tlm::tlm_generic_payload otrans;
      otrans.set_command(tlm::TLM_READ_COMMAND);
      otrans.set_address(address);
      otrans.set_data_length(len);
      otrans.set_data_ptr(dataPtr);
      iSocket->transport_dbg(otrans);
      // spdlog::info("read from downstream");
    }

    address += len;
    dataPtr += len;
    remaining -= len;
  }

  return trans.get_data_length();
}

void Cache::reset() {
  if (!pwrOn.read()) {
    for (auto &s : m_sets) {
      s.reset();
    }
    m_nDirtyLines = 0;
    m_updateNDirtyLinesEvent.notify(SC_ZERO_TIME);
  }
}

void Cache::powerOffChecks() {
  if (m_nDirtyLines > 0) {
    spdlog::warn(
        "{} has {:d} dirty lines at power off, memory may be corrupted!",
        this->name(), m_nDirtyLines);
  }
}

void Cache::flush() {
  wait(SC_ZERO_TIME); // Wait for sim start
  while (1) {
    wait(doFlush.posedge_event());
    spdlog::info("{}::flush flushing cache with {:d} modified lines",
                 this->name(), m_nDirtyLines);
    while (doFlush.read()) {
      sc_time delay = SC_ZERO_TIME;
      // Find & Write back dirty lines
      if (m_nDirtyLines == 0) {
        spdlog::warn(
            "{}::flush doFlush asserted, but there are no dirty lines.",
            this->name());
        break;
      }
      bool done = false;

      // Generate random sequence of set indeces
      auto setIdx = std::vector<int>(m_sets.size(), 0);
      for (unsigned i = 0; i < setIdx.size(); i++) {
        setIdx[i] = i;
      }
      std::random_shuffle(setIdx.begin(), setIdx.end());

      // Generate random sequence of line indeces
      auto lineIdx = std::vector<int>(m_sets[0].lines.size(), 0);
      for (unsigned i = 0; i < lineIdx.size(); i++) {
        lineIdx[i] = i;
      }
      std::random_shuffle(lineIdx.begin(), lineIdx.end());

      // Flush
      for (const auto &i : setIdx) {
        const uint32_t idx = i << m_nOffsetBits;
        for (const auto &j : lineIdx) {
          auto &line = m_sets[i].lines[j];
          if (line.dirty && line.valid) {
            writeLine(line, idx, delay);
            done = true;
            break;
          }
        }
        if (done) {
          break;
        }
      }
      wait(delay);
    }
  }
}

void Cache::debugFlushAndInvalidate() {
  if (m_nDirtyLines != 0) {
    spdlog::info("{}::debugFlush flushing cache", this->name());
  }
  // Find & Write back dirty lines
  for (unsigned i = 0; i < m_sets.size(); i++) {
    uint32_t idx = i << m_nOffsetBits;
    for (auto &line : m_sets[i].lines) {
      if (line.dirty && line.valid) {
        debugWriteLine(line, idx);
      }
      line.valid = false;
    }
  }
}

void Cache::writeLine(CacheLine &line, const uint32_t addr, sc_time &delay) {
  sc_assert(line.dirty); // Don't write back clean lines
  sc_assert(line.valid); // Don't write back invalid lines
  tlm::tlm_generic_payload trans;
  auto wbaddr = writeBackAddress(line, index(addr));
  sc_assert(wbaddr == line.addr); // Sanity check
  trans.set_address(wbaddr);
  trans.set_data_length(m_lineWidth);
  trans.set_data_ptr(&line.data[0]);
  trans.set_command(tlm::TLM_WRITE_COMMAND);
  iSocket->b_transport(trans, delay);
  if (line.dirty == true) {
    m_nDirtyLines--;
    m_updateNDirtyLinesEvent.notify(SC_ZERO_TIME);
  }
  line.dirty = false;

  // Report events
  // Entire cache line is read (to write it downstream)
  powerModelPort->reportEvent(m_nBytesReadEventId, m_lineWidth);
}

void Cache::debugWriteLine(CacheLine &line, const uint32_t addr) {
  sc_assert(line.dirty); // Don't write back clean lines
  sc_assert(line.valid); // Don't write back invalid lines
  auto wbaddr = writeBackAddress(line, index(addr));
  sc_assert(wbaddr == line.addr); // Sanity check

  tlm::tlm_generic_payload trans;
  trans.set_address(wbaddr);
  trans.set_data_length(m_lineWidth);
  trans.set_data_ptr(&line.data[0]);
  trans.set_command(tlm::TLM_WRITE_COMMAND);

  iSocket->transport_dbg(trans);
  line.dirty = false;
}

void Cache::readLine(const uint32_t addr, CacheLine &line, sc_time &delay) {
  sc_assert(!line.dirty); // Don't overwrite dirty lines

  tlm::tlm_generic_payload trans;
  trans.set_address(addr & (~m_offsetMask));
  line.addr = addr & (~m_offsetMask);
  trans.set_data_length(m_lineWidth);
  trans.set_data_ptr(&line.data[0]);
  trans.set_command(tlm::TLM_READ_COMMAND);
  iSocket->b_transport(trans, delay);
  line.tag = tag(addr);
  line.valid = true;
  line.dirty = false;

  // Record events
  // Entire line is overwritten, and  the tag is updated
  powerModelPort->reportEvent(m_tagWriteBitEventId, m_nTagBits);
  powerModelPort->reportEvent(m_nBytesWrittenEventId, m_lineWidth);
}

int Cache::countDirtyLines() const {
  int result = 0;
  for (const auto &set : m_sets) {
    for (const auto &line : set.lines) {
      result += line.dirty;
    }
  }
  return result;
}

void Cache::updatePowerState() {
  if (pwrOn.read() == true) {
    powerModelPort->reportState(m_onStateId);
  } else {
    powerModelPort->reportState(m_offStateId);
  }
}

CacheLine &Cache::findLine(const unsigned address) {
  auto &set = m_sets[index(address)];
  const auto n = set.findLine(tag(address));
  if (n >= 0) { // Found
    return set.lines[n];
  } else {
    throw std::runtime_error(
        fmt::format(FMT_STRING("CacheLine at 0x{:08x} not found."), address));
  }
}

std::ostream &operator<<(std::ostream &os, const Cache &rhs) {
  os << "Cache: " << rhs.name();
  os << "\nnDirtyLines " << rhs.nDirtyLines;
  os << "\ndoFlush " << rhs.doFlush;
  os << "\nContent";
  os << "\nWBADDR     ID L  TAG        V D DATA\n";
  for (unsigned i = 0; i < rhs.m_sets.size(); i++) {
    for (unsigned j = 0; j < rhs.m_sets[i].lines.size(); j++) {
      const auto &line = rhs.m_sets[i].lines[j];
      std::string s =
          fmt::format("0x{:08x} {:02d} {:02d} 0x{:08x} {:1d} {:1d} ", line.addr,
                      i, j, line.tag, line.valid, line.dirty);
      os << s << "[" << std::hex;
      for (unsigned k = 0; k < line.data.size(); k++) {
        os << "0x" << std::hex << static_cast<unsigned>(line.data[k]);
        if (k < line.data.size() - 1) {
          os << ",";
        }
      }
      os << "]\n";
    }
  }
  return os;
}
