/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "ps/EventLog.hpp"

class GenericMemory : public BusTarget {
  SC_HAS_PROCESS(GenericMemory);

 public:
  /* ------ Types ------ */

  /* ------ Public methods ------ */
  GenericMemory(sc_core::sc_module_name name, unsigned startAddress,
                unsigned endAddress);

  /**
   * @brief reset Do nothing, i.e. models nonvolatile memory by default.
   */
  virtual void reset(void) override {}

  /**
   * @brief b_transport Blocking reads and writes
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /**
   * @brief size return size of memory in bytes
   * @retval size of memory in bytes
   */
  virtual int size() const;

  /**
   * @brief transport_dbg Provide memory access without affecting simulation
   * time
   * @param trans
   * @return Number of bytes transfered.
   */
  virtual unsigned int transport_dbg(tlm::tlm_generic_payload &trans) override;

  virtual void end_of_elaboration() override;

 protected:
  std::unique_ptr<uint8_t[]> mem;  // Pointer to emulated memory
  const size_t m_capacity;         // Memory capacity (bytes)
  EventLog::eventId m_nBytesReadEventId_old;
  EventLog::eventId m_nBytesWrittenEventId_old;

  int m_nBytesWrittenEventId;
  int m_nBytesReadEventId;
};
