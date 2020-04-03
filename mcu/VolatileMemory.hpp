/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/GenericMemory.hpp"
#include "ps/EventLog.hpp"

/**
 * @brief The VolatileMemory class Implements overwriting of memory at reset
 */
class VolatileMemory : public GenericMemory {
 public:
  SC_HAS_PROCESS(VolatileMemory);

  /* ------ Public methods ------ */
  VolatileMemory(sc_core::sc_module_name name, unsigned startAddress,
                 unsigned endAddress, sc_core::sc_time delay)
      : GenericMemory(name, startAddress, endAddress, delay) {
    // Call reset method on poweron
    SC_METHOD(reset);
    sensitive << pwrOn;
  }

 private:
  virtual void reset() override {
    if (pwrOn.read()) {
      // Write arbitrary value to memory
      for (unsigned int i = 0; i < m_capacity; i++) {
        mem[i] = 0xAA;
      }
    }
  }
};
