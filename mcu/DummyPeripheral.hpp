/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>
#include <tlm>
#include <vector>
#include "mcu/BusTarget.hpp"

/**
 * @brief The DummyPeripheral class. Initialised with static read-values.
 * The data in write-transactions to the dummy peripheral are simply discarded.
 */

class DummyPeripheral : public BusTarget {
 private:
  std::vector<unsigned char> retvals;

 public:
  DummyPeripheral(const sc_core::sc_module_name name,
                  const std::vector<unsigned char> retvals,
                  const unsigned startAddress, const unsigned endAddress,
                  const sc_core::sc_time delay);

  DummyPeripheral(const sc_core::sc_module_name name,
                  const unsigned startAddress, const unsigned endAddress,
                  const sc_core::sc_time delay);

  /**
   * @brief reset do nothing.
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
   * @brief transport_dbg Read/write registers without affecting sim time
   * @param trans
   * @return
   */
  virtual unsigned int transport_dbg(tlm::tlm_generic_payload &trans) override;
};
