/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "utilities/Config.hpp"
#include <algorithm>
#include <iostream>
#include <string>
#include <systemc-ams>
#include <systemc>

SCA_TDF_MODULE(ConstantCurrentSupply) {
  // Consume voltage
  sca_tdf::sca_in<double> v;

  // Produce contant current
  sca_tdf::sca_out<double> i;

  void set_attributes() { set_timestep(m_timestep); }

  void initialize(){};

  void processing() {
    if ((v.read() + m_maxStepSize) < m_voltageLimit) {
      i.write(m_currentSetpoint);
    } else {
      i.write(0.0);
    }
  }

  void ac_processing(){};

  SCA_CTOR(ConstantCurrentSupply) {
    m_currentSetpoint = Config::get().getDouble("SupplyCurrentLimit");
    m_voltageLimit = Config::get().getDouble("SupplyVoltageLimit");
    m_timestep = sc_core::sc_time::from_seconds(
        Config::get().getDouble("PowerModelTimestep"));
    m_maxStepSize =
        m_timestep.to_seconds() *
        (m_currentSetpoint / Config::get().getDouble("CapacitorValue"));
  };

private:
  double m_currentSetpoint;    // [Ampere]
  double m_voltageLimit;       // [Volt]
  double m_maxStepSize;        // [Volt] Handy to avoid overshoot
  sc_core::sc_time m_timestep; // Evaluation timestep
};

SCA_TDF_MODULE(ConstantPowerSupply) {
  // Consume voltage
  sca_tdf::sca_in<double> v;

  // Produce current
  sca_tdf::sca_out<double> i;

  void set_attributes() { set_timestep(m_timestep); }

  void initialize(){};

  void processing() {
    auto crntVoltage = v.read();
    if (crntVoltage < m_voltageLimit) {
      i.write(std::min(m_currentLimit, m_powerSetpoint / crntVoltage));
    } else {
      i.write(0.0);
    }
  }

  void ac_processing(){};

  SCA_CTOR(ConstantPowerSupply) {
    m_powerSetpoint = Config::get().getDouble("PowerSupplyPower");
    m_voltageLimit = Config::get().getDouble("SupplyVoltageLimit");
    m_currentLimit = Config::get().getDouble("SupplyCurrentLimit");
    m_timestep = sc_core::sc_time::from_seconds(
        Config::get().getDouble("PowerModelTimestep"));
  };

private:
  double m_powerSetpoint;      // [Watt]
  double m_currentLimit;       // [Ampere]
  double m_voltageLimit;       // [Volt]
  sc_core::sc_time m_timestep; // Evaluation timestep
};
