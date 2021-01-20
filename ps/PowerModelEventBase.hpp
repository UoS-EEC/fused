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

/**
 * Abstract base class for power model events.
 */
class PowerModelEventBase {
 public:
  //! Constructor
  PowerModelEventBase(const std::string name_) : name(name_) {}

  /**
   * @brief calculateEnergy calculate event energy adjusted for supply voltage.
   */
  virtual double calculateEnergy(double supplyVoltage) const = 0;

  /**
   * @brief toString return a one-line string for debug/info print.
   */
  virtual std::string toString() const = 0;

  /* Public constants */
  const std::string name;
};
