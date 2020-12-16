/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <memory>
#include <systemc>
#include "ps/PowerModelEventBase.hpp"

class PowerModelEventChannelOutIf : public virtual sc_core::sc_interface {
 public:
  virtual int registerEvent(std::unique_ptr<PowerModelEventBase> eventPtr) = 0;

  virtual void write(const int eventId, const int n = 1) = 0;
};

class PowerModelEventChannelInIf : public virtual sc_core::sc_interface {
 public:
  virtual int pop(const int eventId) = 0;

  virtual double popEnergy(double supplyVoltage) = 0;

  virtual double popEnergy(const int eventId, double supplyVoltage) = 0;

  virtual size_t size() const = 0;
};

// Typedef of ports for convenience
typedef sc_core::sc_port<PowerModelEventChannelOutIf, 0,
                         sc_core::SC_ONE_OR_MORE_BOUND>
    eventOutPort;

typedef sc_core::sc_port<PowerModelEventChannelInIf, 0,
                         sc_core::SC_ONE_OR_MORE_BOUND>
    eventInPort;
