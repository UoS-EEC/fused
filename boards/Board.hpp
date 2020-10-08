/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>
#include "mcu/Microcontroller.hpp"

/**
 * @brief Board base class for PCB-level models in Fused.
 *
 */
SC_MODULE(Board) {
  SC_CTOR(Board) {}

  virtual Microcontroller& getMicrocontroller() = 0;
};
