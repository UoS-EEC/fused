/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <systemc>

class DynamicEnergyIf : public sc_core::sc_interface {
 public:
  virtual double read() = 0;
  virtual void write(double) = 0;
  virtual const sc_core::sc_event &read_event() const = 0;
  virtual const sc_core::sc_event &write_event() const = 0;
};
