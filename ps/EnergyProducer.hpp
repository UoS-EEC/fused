/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>
#include "ps/DynamicEnergyChannel.hpp"

class EnergyProducer : public sc_core::sc_module {
 public:
  /* ------ Ports ------ */
  //! Dynamic/instantaneous energy Production
  sc_core::sc_port<DynamicEnergyIf, 1, sc_core::SC_ONE_OR_MORE_BOUND>
      dynamicProduction{"dynamicProduction"};

  //! Static current Production
  sc_core::sc_out<float> staticProduction{"staticProduction"};

  //! Supply voltage
  sc_core::sc_in<float> vcc{"vcc"};

  /* ------ Public methods ------ */
  explicit EnergyProducer(sc_core::sc_module_name name)
      : sc_core::sc_module(name) {}

  /**
   * @brief produceDynamic Instantaneously produce a quantity of energy
   * @param energy in mJ
   */
  inline void produceDynamic(double energy);

  /**
   * @brief setStaticProduction Set static current production.
   * @param current current in mA
   */
  inline void setStaticProduction(double current);

 protected:
};
