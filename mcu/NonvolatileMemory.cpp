/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <iterator>
#include "mcu/NonvolatileMemory.hpp"
#include "utilities/Config.hpp"

using namespace sc_core;

NonvolatileMemory::NonvolatileMemory(sc_module_name name, unsigned startAddress,
                                     unsigned endAddress)
    : GenericMemory(name, startAddress, endAddress) {}

void NonvolatileMemory::b_transport(tlm::tlm_generic_payload &trans,
                                    sc_time &delay) {
  sc_time dummyDelay = sc_time(0, SC_NS);  // ignore GenericMemory's delay
  GenericMemory::b_transport(trans, dummyDelay);
  delay += waitStates.read() * systemClk->getPeriod();
}

unsigned int NonvolatileMemory::countSetBitsArray(const uint8_t *arr,
                                                  const size_t N) {
  unsigned res = 0;
  for (size_t i = 0; i < N; i++) {
    res += countSetBits(arr[i]);
  }
  return res;
}

// Kernighan's Algorithm
unsigned int NonvolatileMemory::countSetBits(uint64_t n) {
  unsigned int count = 0;
  while (n) {
    n &= (n - 1);
    count++;
  }
  return count;
}
