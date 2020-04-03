/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>

class EnergyConsumer : public sc_core::sc_module {
 public:
  /* ------ Ports ------ */
  //! Dynamic/instantaneous energy consumption
  sc_core::sc_fifo_out<float> dynamicConsumption{"dynamicConsumption"};

  //! Static current consumption
  sc_core::sc_out<float> staticConsumption{"staticConsumption"};

  //! Supply voltage
  sc_core::sc_in<float> vcc{"vcc"};

  /* ------ Public methods ------ */
  EnergyConsumer(sc_core::sc_module_name name);

  /**
   * @brief consumeDynamic Instantaneously consume a quantity of energy
   * @param energy in J
   */
  inline void consumeDynamic(float energy);

  /**
   * @brief setStaticConsumption Set static current consumption.
   * @param current current in A
   */
  inline void setStaticConsumption(float current);

 protected:
  /* ------ Signals ------ */
  //    sc_core::sc_fifo<float> m_dynamicConsumption{"m_dynamicConsumption"};
  sc_core::sc_signal<float> m_staticConsumption{"m_staticConsumption"};
};
