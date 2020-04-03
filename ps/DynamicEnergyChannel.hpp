/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <systemc>
#include "ps/DynamicEnergyIf.hpp"

class DynamicEnergyChannel : public DynamicEnergyIf,
                             public sc_core::sc_channel {
 public:
  explicit DynamicEnergyChannel(sc_core::sc_module_name nm)
      : sc_core::sc_channel(nm), m_val{0.0f} {}

  double read() override {
    double tmp = m_val;
    m_val = 0.0;
    readEvent.notify(sc_core::SC_ZERO_TIME);
    return tmp;
  }

  void write(double v) override {
    m_val += v;
    writeEvent.notify(sc_core::SC_ZERO_TIME);
  }

  virtual const sc_core::sc_event &read_event() const override {
    return readEvent;
  }

  virtual const sc_core::sc_event &write_event() const override {
    return writeEvent;
  }

  virtual const sc_core::sc_event &default_event() const override {
    return writeEvent;
  }

 private:
  /* ------ Private variables ------ */
  double m_val;
  sc_core::sc_event readEvent{"readEvent"};
  sc_core::sc_event writeEvent{"writeEvent"};
};
