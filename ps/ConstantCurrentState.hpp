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
#include "ps/PowerModelStateBase.hpp"
#include "utilities/Config.hpp"

/**
 * Power model state for states that have a constant current consumption
 * regardless of supply voltage and clock frequency.
 */
class ConstantCurrentState : public PowerModelStateBase {
 public:
  //! Constructor
  ConstantCurrentState(const std::string name, double current_)
      : PowerModelStateBase(name, -1), current(current_) {}

  ConstantCurrentState(const std::string name)
      : PowerModelStateBase(name, -1),
        current(Config::get().contains(name) ? Config::get().getDouble(name)
                                             : 0.0) {}

  virtual double calculateCurrent(
      [[maybe_unused]] const double supplyVoltage,
      [[maybe_unused]] const double clockFrequency) const override {
    return current;
  }

  /* Public constants */
  const double current;
};
