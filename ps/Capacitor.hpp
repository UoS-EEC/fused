/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include <systemc-ams>
#include <systemc>

/*
 * Description:
 *
 * A simple capacitor model that computes stored voltage based on its
 * capacitance, input current, and output current.
 *
 * The output voltage is initialized to a specified value, and is delayed by one
 * sample to ease TDF scheduling.
 *
 */

SCA_TDF_MODULE(Capacitor) {
  // Consume input and output current
  sca_tdf::sca_in<double> i_in{"i_in"};
  sca_tdf::sca_in<double> i_out{"i_out"};

  // Produce output voltage
  sca_tdf::sca_out<double> v{"v"};

  // Constructor
  Capacitor(const sc_core::sc_module_name name, const double capacitance,
            const double initialVoltage = 0.0)
      : sca_tdf::sca_module(name), m_capacitance(capacitance),
        m_crntVoltage(initialVoltage){};

  // Set a 1-sample delay on the output
  void set_attributes() { v.set_delay(1); };

  // Initialization
  void initialize() {
    v.initialize(m_crntVoltage);
    m_timestep = get_timestep().to_seconds();
  };

  // Evaluation
  void processing() {
    m_crntVoltage += m_timestep * (i_in.read() - i_out.read()) / m_capacitance;
    v.write(m_crntVoltage);
  }

private:
  double m_capacitance;
  double m_crntVoltage;
  double m_timestep;
};
