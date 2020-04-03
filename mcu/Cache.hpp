/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <iostream>
#include <list>
#include <string>
#include <systemc>
#include <tlm>
#include <vector>
#include "mcu/BusTarget.hpp"
#include "mcu/CacheReplacementPolicies.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"

/* Cache line */
struct CacheLine {
  bool valid{false};
  bool dirty{false};
  unsigned tag{0};
  std::vector<uint8_t> data;
  CacheLine(const int lineWidth) : data(lineWidth, 0) {}
  void reset() {
    valid = false;
    dirty = false;
    tag = 0;
    for (auto &d : data) {
      d = 0xAA;
    }
  }
};

/* Cache set */
class CacheSet {
 public:
  /* ------ Public methods ------ */
  CacheSet(const int nLines, const int lineWidth,
           CacheReplacementIf *replacementPolicy);
  int findLine(const unsigned tag);
  unsigned miss(const unsigned tag);
  void hit(const unsigned tag);
  void reset();

  /* ------ Public variables ------ */
  CacheReplacementIf *replacementPolicy;
  std::vector<CacheLine> lines;
};

class Cache : public BusTarget, public tlm::tlm_bw_transport_if<> {
  SC_HAS_PROCESS(Cache);

 public:
  /* ------ Ports ------ */
  tlm::tlm_initiator_socket<> iSocket{"iSocket"};  //! Memory (master) port

  /* ------ Submodules ------ */

  /* ------ Public methods ------ */
  /**
   * @brief Cache constructor
   */
  Cache(const sc_core::sc_module_name name, const unsigned startAddress,
        const unsigned endAddress, const sc_core::sc_time delay);

  /**
   * @brief b_transport Blocking reads and writes. Overridden to include
   * cache
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;
  /**
   * @brief transport_dbg forward directly to memory
   * @param trans
   * @param delay
   */
  unsigned int transport_dbg(tlm::tlm_generic_payload &trans) override;

  /**
   * @brief << operator debug printout
   */
  friend std::ostream &operator<<(std::ostream &os, const Cache &rhs);

 private:
  /* ------ Constants ------ */
  /* ------ Types ------ */
  /* ------ Private variables ------ */
  std::vector<CacheSet> m_sets;
  int m_lineWidth;
  int m_nSets;
  int m_nLines;
  int m_nOffsetBits;
  int m_nIdBits;
  unsigned m_offsetMask;
  unsigned m_idMask;
  unsigned m_tagMask;

  enum write_policy {
    WP_WRITE_THROUGH,
    WP_WRITE_AROUND,
    WP_WRITE_BACK
  } m_writePolicy;

  EventLog::eventId m_readMissEvent;
  EventLog::eventId m_readHitEvent;
  EventLog::eventId m_writeMissEvent;
  EventLog::eventId m_writeHitEvent;
  EventLog::eventId m_nBytesReadEvent;
  EventLog::eventId m_nBytesWrittenEvent;

  /* ------- Private methods ------ */

  /**
   * @brief reset Reset to power-on defaults (clear state).
   */
  virtual void reset() override;

  /**
   * @brief writeLine write a line to memory (master port) and update line state
   * accordingly
   * @param line line to write
   * @param addr address (used for the index field of the writeback address)
   * @param delay accumulative access delay
   */
  void writeLine(CacheLine &line, const uint32_t addr, sc_core::sc_time &delay);

  /**
   * @brief readLine read a line from memory (master port), and update line
   * state accordingly
   * @param addr address to write line to.
   * @param line line to write
   * @param delay accumulative access delay
   */
  void readLine(const uint32_t addr, CacheLine &line, sc_core::sc_time &delay);

  /**
   * @brief Get index of address
   * @param addr address
   */
  unsigned int index(const unsigned addr) const {
    return (addr & m_idMask) >> m_nOffsetBits;
  }

  /**
   * @brief Get tag of address
   * @param addr address
   */
  unsigned int tag(const unsigned addr) const {
    return (addr & m_tagMask) >> (m_nOffsetBits + m_nIdBits);
  }

  /**
   * @brief Get offset of address
   * @param addr address
   */
  unsigned int offset(const unsigned addr) const {
    return (addr & m_offsetMask);
  }

  /* ------ Dummy methods ------ */
  // Dummy method:
  [[noreturn]] void invalidate_direct_mem_ptr(
      sc_dt::uint64 start_range[[maybe_unused]],
      sc_dt::uint64 end_range[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "invalidate_direct_mem_ptr not implement");
    exit(1);
  }

      // Dummy method:
      [[noreturn]] tlm::tlm_sync_enum
      nb_transport_bw(tlm::tlm_generic_payload &trans[[maybe_unused]],
                      tlm::tlm_phase &phase[[maybe_unused]],
                      sc_core::sc_time &delay[[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "nb_transport_bw is not implemented");
    exit(1);
  }
};
