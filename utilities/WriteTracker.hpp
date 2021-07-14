/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "include/peripheral-defines.h"
#include "mcu/BusTarget.hpp"
#include "utilities/Config.hpp"
#include <algorithm>
#include <iostream>
#include <spdlog/spdlog.h>
#include <string>
#include <systemc>
#include <tlm>
#include <vector>

/*
 * DESCRIPTION
 * WriteTracker tracks which blocks of memory that have been written to, and
 * provides an interface to access dirty bits that are stored in a memory.
 *
 * Implements a control register and a series of registers that show whether a
 * block of memory has been written to since this module was enabled.
 *
 *
 */

class WriteTracker : public BusTarget,
                     public tlm::tlm_analysis_if<tlm::tlm_generic_payload> {

public:
  /* ------ Ports ------ */
  /* ------ Constants  ------ */

  struct RegisterAddress {
    /**
     * Control register
     *  Fields:
     *     0:           Enable
     *     1:           Clear
     *     2..31:       Undefined
     */
    static const unsigned Control = 0;

    /**
     * Dirty register
     *   Fields:
     *     Each bit indicates whether the corresponding block of monitored
     *     memory is dirty (has been written to while this module is enabled)
     */
    static const unsigned Dirty = 4;
  };

  struct BitMask {
    //! Enable monitoring
    static const unsigned Control_Enable = (1u << 0);

    //! Clear dirty bits
    static const unsigned Control_Clear = (1u << 1);
  };

  /* ------ Public methods ------ */

  /**
   * @brief WriteTracker Constructor
   * @param name
   * @param startAddress start address of this peripheral's register file
   * @param monitorEndAddress end address of monitored memory
   * @param blockSize monitoring granularity in bytes. Monitored memory space
   * must be aligned with the block size.
   */
  WriteTracker(const sc_core::sc_module_name name, const unsigned startAddress,
               const unsigned monitorEndAddress, const int blockSize)
      : BusTarget(name, startAddress,
                  startAddress +
                      4 * (1 + (monitorEndAddress + 1) / blockSize / 32)),
        m_monitorEndAddress(monitorEndAddress), m_blockSize(blockSize) {
    // Build register file
    m_regs.addRegister(RegisterAddress::Control, 0);

    // Check that monitored space is aligned with block size
    sc_assert((monitorEndAddress + 1) % blockSize == 0);

    // Each register covers 32 blocks (1 block per bit)
    const int nBlocks = (monitorEndAddress + 1) / blockSize;
    const int nRegs = nBlocks / 32;

    for (int i = 0; i < nRegs; ++i) {
      m_regs.addRegister(RegisterAddress::Dirty + 4 * i, 0,
                         RegisterFile::AccessMode::READ);
    }
  }

  /**
   * @brief write called via the analysis port interface for each transaction.
   * This function records the blocks of memory that have been written to.
   *
   */
  void write(const tlm::tlm_generic_payload &trans) override {
    auto addr = trans.get_address();
    if (!m_enable || trans.get_command() != tlm::TLM_WRITE_COMMAND ||
        addr > m_monitorEndAddress) {
      return;
    }

    while (addr < trans.get_address() + trans.get_data_length()) {
      if (addr > m_monitorEndAddress) {
        ++addr;
        continue;
      }
      const int blockNumber = addr / m_blockSize;
      const int registerAddress =
          RegisterAddress::Dirty + 4 * (blockNumber / 32);
      const int blockBitNumber = blockNumber % 32;
      m_regs.setBit(registerAddress, blockBitNumber, /*force=*/true);
      ++addr;
    }
  }

  /**
   * @brief b_transport bus interface
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override {
    BusTarget::b_transport(trans, delay);
    const auto addr = trans.get_address();

    if (trans.get_command() == tlm::TLM_WRITE_COMMAND) {
      const auto data = m_regs.read(addr);
      switch (addr) {
      case RegisterAddress::Control:
        m_enable = data & BitMask::Control_Enable;
        if (data & BitMask::Control_Clear) {
          m_regs.reset();
          if (m_enable) {
            m_regs.setBitMask(RegisterAddress::Control,
                              BitMask::Control_Enable);
          }
        }
        break;
      }
    }
  }

  /**
   * @brief reset
   */
  void reset() override {
    m_regs.reset();
    m_enable = false;
  }

private:
  /* ------ Types ------ */
  /* ------ Private variables ------ */
  bool m_enable{false};
  const unsigned m_monitorEndAddress; //! End of monitored address space
  const int m_blockSize;              //! Monitoring granularity

  /* ------ Private methods ------ */
};
