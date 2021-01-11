/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <iostream>
#include <string>

/**
 * Abstract base class for power model states.
 */
class PowerModelStateBase {
 public:
  //! Constructor
  PowerModelStateBase(const std::string name_) : name(name_) {}

  /**
   * @brief calculateEnergy calculate state current , optionally adjusted for
   * supply voltage.
   */
  virtual double calculateCurrent(double supplyVoltage) const = 0;

  /**
   * @brief toString return a one-line string for debug/info print.
   */
  virtual std::string toString() const = 0;

  /* Public constants */
  const std::string name;
};
