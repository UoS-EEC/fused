/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <iostream>
#include <string>
#include "ps/PowerModelEventBase.hpp"

/**
 * Power model event for events that have a constant energy consumption
 * regardless of supply voltage.
 */
class ConstantEnergyEvent : public PowerModelEventBase {
 public:
  //! Constructor
  ConstantEnergyEvent(const std::string name, double energy_)
      : PowerModelEventBase(name, -1), energy(energy_) {}

  virtual double calculateEnergy([
      [maybe_unused]] const double supplyVoltage) const override {
    return energy;
  }

  /* Public constants */
  const double energy;
};
