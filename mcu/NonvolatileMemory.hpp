/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stdint.h>
#include <array>
#include <iostream>
#include <list>
#include <string>
#include "mcu/CacheReplacementPolicies.hpp"
#include "mcu/GenericMemory.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Config.hpp"

class NonvolatileMemory : public GenericMemory {
 public:
  /* ------ Ports ------ */
  sc_core::sc_in<unsigned int> waitStates{"waitStates"};

  /* ------ Public methods ------ */
  /**
   * @brief NonvolatileMemory constructor
   */
  explicit NonvolatileMemory(sc_core::sc_module_name name,
                             unsigned startAddress, unsigned endAddress,
                             sc_core::sc_time delay);

  /**
   * @brief b_transport Blocking reads and writes. Overridden to set delay
   * according to wait states
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

 private:
  /* ------ Constants ------ */
  /* ------ Types ------ */
  /* ------ Private variables ------ */
  /* ------- Private methods ------ */
  unsigned int countSetBits(uint64_t n);
  unsigned int countSetBitsArray(const uint8_t *arr, const size_t N);
};
