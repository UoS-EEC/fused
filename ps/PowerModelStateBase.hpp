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
 * Abstract base class for power model states.
 */
class PowerModelStateBase {
 public:
  //! Constructor
  PowerModelStateBase(const std::string name_, const int id_)
      : name(name_), id(id_) {}

  /**
   * @brief calculateEnergy calculate state current , optionally adjusted for
   * supply voltage.
   */
  virtual double calculateCurrent(double supplyVoltage) const = 0;

  /* Public constants */
  const std::string name;
  int id;  // TODO find a way to make this const
};
