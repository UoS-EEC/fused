/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <spdlog/fmt/fmt.h>
#include <stdint.h>
#include <iostream>
#include <string>
#include "ps/PowerModelEventBase.hpp"
#include "utilities/Config.hpp"

/**
 * Power model event for events that have a constant energy consumption
 * regardless of supply voltage.
 */
class ConstantEnergyEvent : public PowerModelEventBase {
 public:
  //! Constructor
  ConstantEnergyEvent(const std::string name, double energy_)
      : PowerModelEventBase(name), energy(energy_) {}

  ConstantEnergyEvent(const std::string name)
      : PowerModelEventBase(name),
        energy(Config::get().contains(name) ? Config::get().getDouble(name)
                                            : 0.0) {}

  virtual double calculateEnergy([
      [maybe_unused]] const double supplyVoltage) const override {
    return energy;
  }

  virtual std::string toString() const override {
    return fmt::format(
        FMT_STRING("{:s} <ConstantEnergyEvent> energy={:.6f} nJ"), name,
        energy * 1e9);
  }

  /* Public constants */
  const double energy;
};
