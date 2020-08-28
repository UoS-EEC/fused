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
#include "mcu/RegisterFile.hpp"

#define FRCTL_A_SIZE (OFS_GCCTL1 + 2)

class Frctl_a : public BusTarget {
  SC_HAS_PROCESS(Frctl_a);

 public:
  /* ------ Ports ------ */
  sc_core::sc_out<unsigned int> waitStates{"waitStates"};

  /* ------ Methods ------ */
  /**
   * Default constructor, reads address from msp430frxxx.h
   */
  Frctl_a(sc_core::sc_module_name nm);

  /**
   * Full constructor
   */
  Frctl_a(sc_core::sc_module_name nm, unsigned startAddress,
          unsigned endAddress);

  virtual void reset() override;

 private:
  /* ------ Private variables ------ */

  /* ------ Private methods ------ */
  void process();
};
