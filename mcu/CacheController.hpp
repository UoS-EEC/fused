/*
 * Copyright (c) 2020-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include "mcu/BusTarget.hpp"
#include "utilities/Config.hpp"
#include <iostream>
#include <stdint.h>
#include <string>
#include <systemc>
#include <tlm>
#include <vector>

class CacheController : public BusTarget {
  SC_HAS_PROCESS(CacheController);

public:
  /* ------ Ports ------ */
  sc_core::sc_out<bool> doFlush{"doFlush"};
  sc_core::sc_in<int> nDirtyLines{"nDirtyLines"};

  /* ------ Public Methods ------ */
  /**
   * @brief CacheController constructor.
   */
  CacheController(const sc_core::sc_module_name nm,
                  const unsigned startAddress);

  /**
   * @brief reset Reset to power-on defaults.
   */
  virtual void reset() override;

  /**
   * @brief set up methods/threads after module construction complete.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const CacheController &rhs);

private:
  /* ------ Private methods ------ */
  /**
   * @brief process main process loop
   */
  void process();
};
