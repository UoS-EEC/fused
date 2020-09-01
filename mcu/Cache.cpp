/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <cmath>
#include <string>
#include <systemc>
#include <tlm>
#include "mcu/Cache.hpp"
#include "mcu/CacheReplacementPolicies.hpp"
#include "utilities/Config.hpp"

using namespace sc_core;

CacheSet::CacheSet(const int nLines, const int lineWidth,
                   CacheReplacementIf *replacementPolicy)
    : replacementPolicy(replacementPolicy), lines(nLines, lineWidth) {}

int CacheSet::findLine(const unsigned tag) {
  for (unsigned int i = 0; i < lines.size(); i++) {
    if (lines[i].tag == tag && lines[i].valid) {
      return i;  // Cache hit
    }
  }
  return -1;  // Cache miss
}

void CacheSet::reset() {
  for (auto &l : lines) {
    l.reset();
  }
  replacementPolicy->reset();
}

Cache::Cache(const sc_module_name name, const unsigned startAddress,
             const unsigned endAddress)
    : BusTarget(name, startAddress, endAddress) {
  iSocket.bind(*this);

  // Config
  std::string strname = this->name();
  auto cfg = m_lineWidth = Config::get().getUint(strname + ".CacheLineWidth");
  m_nSets = Config::get().getUint(strname + ".CacheNSets");
  m_nLines = Config::get().getUint(strname + ".CacheNLines");
  m_nOffsetBits = static_cast<int>(log2(m_lineWidth));
  m_offsetMask = m_lineWidth - 1;
  m_nIdBits = static_cast<int>(log2(m_nSets));
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
    } else if (setCfg == "LFU") {
      m_sets.push_back(CacheSet(m_nLines, m_lineWidth,
                                new CacheReplacementLfu(m_nLines, 64)));
    } else if (setCfg == "RoundRobin") {
      m_sets.push_back(CacheSet(m_nLines, m_lineWidth,
                                new CacheReplacementRoundRobin(m_nLines)));
    } else if (setCfg == "PseudoRandom") {
      m_sets.push_back(CacheSet(m_nLines, m_lineWidth,
                                new CacheReplacementPseudoRandom(m_nLines)));
    } else {
      spdlog::error("FRAM Cache set: invalid replacement policy {}", setCfg);
      SC_REPORT_ERROR("FRAM Cache set", "Invalid replacement policy.");
    }
  }

  // Register events
  m_readMissEvent = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " read miss");
  m_readHitEvent = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " read hit");
  m_writeMissEvent = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " write miss");
  m_writeHitEvent = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " write hit");
  m_nBytesReadEvent = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " bytes read");
  m_nBytesWrittenEvent = EventLog::getInstance().registerEvent(
      std::string(this->name()) + " bytes written");

  // Methods & threads
  SC_METHOD(reset);
  sensitive << pwrOn;
};

void Cache::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {
  auto addr = trans.get_address();
  uint8_t *dataPtr = trans.get_data_ptr();
  auto len = trans.get_data_length();

  // Check alignment to cache line
  if (offset(addr) > offset(addr + len - 1)) {
    spdlog::error(
        "Transaction with address  0x{:08x} and length {:d} unaligned to cache "
        "line.",
        addr, len);
    SC_REPORT_FATAL(this->name(), "Access unaligned to cache line.");
  }

  int lineIdx = m_sets[index(addr)].findLine(tag(addr));
  assert(lineIdx >= 0 ? lineIdx < m_nLines : 1);
  bool hit = lineIdx >= 0;
  if (hit) {
    m_sets[index(addr)].replacementPolicy->hit(
        lineIdx, trans.get_command() == tlm::TLM_WRITE_COMMAND);
  } else {  // Miss -- pick victim line
    lineIdx = m_sets[index(addr)].replacementPolicy->miss(
        trans.get_command() == tlm::TLM_WRITE_COMMAND);
    assert(lineIdx < m_nLines);
  }

  auto &line = m_sets[index(addr)].lines[lineIdx];

  if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
    m_writeEvent.notify(delay + systemClk->getPeriod());
    m_elog.increment(m_writeEventId);
    m_elog.increment(m_nBytesWrittenEvent, len);

    if (hit) {
      m_elog.increment(m_writeHitEvent);
    } else {
      m_elog.increment(m_writeMissEvent);
    }

    tlm::tlm_generic_payload outputTrans;
    switch (m_writePolicy) {
      case WP_WRITE_AROUND:  // Update memory only
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
      case WP_WRITE_THROUGH:  // Update cache line & memory
        sc_assert(line.dirty == false);
        if (!hit) {
          readLine(addr, line, delay);  // load new line
        }

        // Update cached data
        memcpy(&line.data[offset(addr)], dataPtr, len);

        // Update memory
        outputTrans.set_address(addr);
        outputTrans.set_data_length(len);
        outputTrans.set_data_ptr(dataPtr);
        outputTrans.set_command(tlm::TLM_WRITE_COMMAND);
        iSocket->b_transport(outputTrans, delay);
        break;
      case WP_WRITE_BACK:  // Update cache line only
        if (!hit && line.valid && line.dirty) {
          writeLine(line, addr, delay);  // Write back victim
        }
        if (!hit) {
          readLine(addr, line, delay);  // load new line
        }
        // Update cached data
        memcpy(&line.data[offset(addr)], dataPtr, len);
        line.dirty = true;
        break;
      default:
        SC_REPORT_FATAL(this->name(), "Invalid write policy.");
        break;
    }
  } else if (trans.get_command() == tlm::TLM_READ_COMMAND) {
    m_readEvent.notify(delay + systemClk->getPeriod());
    m_elog.increment(m_readEventId);
    m_elog.increment(m_nBytesReadEvent, trans.get_data_length());

    if (hit) {
      m_elog.increment(m_readHitEvent);
    } else {
      m_elog.increment(m_readMissEvent);
    }

    if (!hit) {
      // Miss -- Fetch data from memory before serving
      if (line.valid && line.dirty) {
        writeLine(line, addr, delay);  // Write back victim first
      }
      readLine(addr, line, delay);  // Fetch new line
    }

    // Return data
    std::memcpy(dataPtr, &line.data[offset(addr)], len);
  } else {
    SC_REPORT_FATAL(this->name(), "Transaction command not supported.");
  }

  delay += systemClk->getPeriod();
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned int Cache::transport_dbg(tlm::tlm_generic_payload &trans) {
  //! TODO: Should make sure that cached values are returned, so that
  // debugger doesn't read stale data. For now, just forwarding accesses
  // to memory
  return iSocket->transport_dbg(trans);
}

void Cache::reset() {
  for (auto &s : m_sets) {
    s.reset();
  }
}

void Cache::writeLine(CacheLine &line, const uint32_t addr, sc_time &delay) {
  sc_assert(line.dirty);  // Don't write back clean lines
  sc_assert(line.valid);  // Don't write back valid lines
  tlm::tlm_generic_payload trans;
  uint32_t wbaddr = ((line.tag << (m_nOffsetBits + m_nIdBits)) |
                     (index(addr) << m_nOffsetBits));
  trans.set_address(wbaddr);
  trans.set_data_length(m_lineWidth);
  trans.set_data_ptr(&line.data[0]);
  trans.set_command(tlm::TLM_WRITE_COMMAND);
  iSocket->b_transport(trans, delay);
  line.dirty = false;
}

void Cache::readLine(const uint32_t addr, CacheLine &line, sc_time &delay) {
  sc_assert(!line.dirty);  // Don't overwrite dirty lines
  tlm::tlm_generic_payload trans;
  trans.set_address(addr & (~m_offsetMask));
  trans.set_data_length(m_lineWidth);
  trans.set_data_ptr(&line.data[0]);
  trans.set_command(tlm::TLM_READ_COMMAND);
  iSocket->b_transport(trans, delay);
  line.tag = tag(addr);
  line.valid = true;
  line.dirty = false;
}

std::ostream &operator<<(std::ostream &os, const Cache &rhs) {
  os << "Cache: " << rhs.name();
  os << "\nContent";
  os << "\nID L  TAG        V D DATA\n";
  for (unsigned i = 0; i < rhs.m_sets.size(); i++) {
    for (unsigned j = 0; j < rhs.m_sets[i].lines.size(); j++) {
      const auto &line = rhs.m_sets[i].lines[j];
      std::string s = fmt::format("{:02d} {:02d} 0x{:08x} {:1d} {:1d} ", i, j,
                                  line.tag, line.valid, line.dirty);
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
