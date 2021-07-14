/*
 * Copyright (c) 2018-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include "mcu/BusTarget.hpp"
#include "mcu/CacheController.hpp"
#include "mcu/CacheReplacementPolicies.hpp"
#include "utilities/Config.hpp"
#include <iostream>
#include <list>
#include <stdint.h>
#include <string>
#include <systemc>
#include <tlm>
#include <vector>

/* Cache line */
struct CacheLine {
  bool valid{false};
  bool dirty{false};
  unsigned tag{0};
  unsigned addr{0};
  std::vector<uint8_t> data;
  CacheLine(const int lineWidth) : data(lineWidth, 0) {}
  void reset() {
    valid = false;
    dirty = false;
    addr = 0;
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
  tlm::tlm_initiator_socket<> iSocket{"iSocket"}; //! Memory (master) port

  /* ------ Submodules ------ */
  CacheController cacheCtrl;

  /* ------ Signals ------ */
  sc_core::sc_signal<int> nDirtyLines{"nDirtyLines", 0};
  sc_core::sc_signal<bool> doFlush{"doFlush", false};

  /* ------ Public methods ------ */
  /**
   * @brief Cache constructor
   */
  Cache(const sc_core::sc_module_name name, const unsigned startAddress,
        const unsigned endAddress, const unsigned ctrlAddress);

  /**
   * @brief b_transport Blocking reads and writes. Overridden to include
   * cache
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;
  /**
   * @brief set up methods, sensitivity, and register power model events and
   * states
   */
  virtual void end_of_elaboration() override;
  /**
   * @brief transport_dbg forward directly to memory
   * @param trans
   */
  unsigned int transport_dbg(tlm::tlm_generic_payload &trans) override;

  /**
   * @brief size return cache capacity in bytes.
   */
  int size() const { return m_lineWidth * m_nLines * m_nSets; }

  /**
   * @brief lineWidth get cache line width
   */
  int lineWidth() const { return m_lineWidth; }

  /**
   * @brief writePolicy get the name of the cache write policy
   */
  int nLines() const { return m_nLines; }

  std::string writePolicy() const {
    return Config::get().getString(std::string(this->name()) +
                                   ".CacheWritePolicy");
  }

  /**
   * @brief writePolicy get the name of the cache replacement policy
   */
  std::string replacementPolicy() const {
    return Config::get().getString(std::string(this->name()) +
                                   ".CacheReplacementPolicy");
  }

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
  int m_nTagBits;
  int m_nDirtyLines{0};
  unsigned m_offsetMask;
  unsigned m_idMask;
  unsigned m_tagMask;
  sc_core::sc_event m_updateNDirtyLinesEvent{"updateNDirtyLinesEvent"};

  enum write_policy {
    WP_WRITE_THROUGH,
    WP_WRITE_AROUND,
    WP_WRITE_BACK
  } m_writePolicy;

  int m_readMissEventId{-1};
  int m_readHitEventId{-1};
  int m_writeMissEventId{-1};
  int m_writeHitEventId{-1};
  int m_nBytesReadEventId{-1};
  int m_nBytesWrittenEventId{-1};
  int m_tagReadBitEventId{-1};
  int m_tagWriteBitEventId{-1};
  int m_onStateId{-1};
  int m_offStateId{-1};

  /* ------- Private methods ------ */

  /**
   * @brief findLine search the entire cache to find the specified line.
   * @retval reference to the line if found
   */
  CacheLine &findLine(unsigned address);

  /**
   * @brief reset Reset to power-on defaults (clear state).
   */
  virtual void reset() override;

  /**
   * @brief updatePowerState Report power state to eventlog.
   */
  virtual void updatePowerState();

  /**
   * @brief powerOffChecks Check status of cache when power is lost. Used to
   * issue warning if cache has dirty state when power is lost, i.e. when memory
   * could be corrupted.
   */
  void powerOffChecks();

  /**
   * @brief flush Flush dirty cache lines, used as an SC_THREAD
   */
  void flush();

  /**
   * @brief debugFlushAndInvalidate Flush and invalidate dirty cache lines using
   * transport_dbg. This is used to clear cache state on debug writes (e.g. when
   * programming main memory).
   */
  void debugFlushAndInvalidate();

  /**
   * @brief updateNDirtyLines update nDirtyLines output signal
   */
  void updateNDirtyLines();

  /**
   * @brief writeLine write a line to memory (master port) and update line state
   * accordingly
   * @param line line to write
   * @param addr address (used for the index field of the writeback address)
   * @param delay accumulative access delay
   */
  void writeLine(CacheLine &line, const uint32_t addr, sc_core::sc_time &delay);

  /**
   * @brief debugWriteLine write a line to memory (master port) and update line
   * state accordingly
   * @param line line to write
   * @param addr address (used for the index field of the writeback address)
   */
  void debugWriteLine(CacheLine &line, const uint32_t addr);

  /**
   * @brief readLine read a line from memory (master port), and update line
   * state accordingly
   * @param addr address to write line to.
   * @param line line to write
   * @param delay accumulative access delay
   */
  void readLine(const uint32_t addr, CacheLine &line, sc_core::sc_time &delay);

  /**
   * @brief countDirtyLines count the total number of dirty lines
   * @retval number of dirty lines
   */
  int countDirtyLines() const;

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

  /**
   * @brief Get write back address of a line at index
   */
  unsigned int writeBackAddress(const CacheLine &line, const unsigned index) {
    return ((line.tag << (m_nOffsetBits + m_nIdBits)) |
            (index << m_nOffsetBits));
  }

  /* ------ Dummy methods ------ */
  // Dummy method:
  [[noreturn]] void invalidate_direct_mem_ptr(sc_dt::uint64 start_range
                                              [[maybe_unused]],
                                              sc_dt::uint64 end_range
                                              [[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "invalidate_direct_mem_ptr not implement");
    exit(1);
  }

  // Dummy method:
  [[noreturn]] tlm::tlm_sync_enum
  nb_transport_bw(tlm::tlm_generic_payload &trans [[maybe_unused]],
                  tlm::tlm_phase &phase [[maybe_unused]],
                  sc_core::sc_time &delay [[maybe_unused]]) {
    SC_REPORT_FATAL(this->name(), "nb_transport_bw is not implemented");
    exit(1);
  }
};
