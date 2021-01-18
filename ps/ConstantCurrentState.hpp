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
#include "ps/PowerModelStateBase.hpp"
#include "utilities/Config.hpp"

/**
 * Power model state for states that have a constant current consumption
 * regardless of supply voltage.
 */
class ConstantCurrentState : public PowerModelStateBase {
 public:
  //! Constructor
  ConstantCurrentState(const std::string name, double current_)
      : PowerModelStateBase(name), current(current_) {}

  /**
   * @brief alternative constructor which attempts to set the current from the
   * config item named "<moduleName> <name>". If the config does not contain
   * that name, 0.0 is used.
   * @param moduleName module name used for finding the current from the config.
   * @param name name of this state.
   */
  ConstantCurrentState(const std::string moduleName, const std::string name)
      : PowerModelStateBase(name),
        current(Config::get().contains(moduleName + " " + name)
                    ? Config::get().getDouble(moduleName + " " + name)
                    : 0.0) {}

  virtual double calculateCurrent([
      [maybe_unused]] const double supplyVoltage) const override {
    return current;
  }

  virtual std::string toString() const override {
    return fmt::format(
        FMT_STRING("<ConstantCurrentState> {:s}: current={:.6} nA"), name,
        current * 1e9);
  }

  /* Public constants */
  const double current;
};
