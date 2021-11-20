/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "ps/PvCell.hpp"
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

SCA_TDF_MODULE(PvCellSupply) {
  // Consume voltage
  sca_tdf::sca_in<double> v;

  // Produce current
  sca_tdf::sca_out<double> i;

  void set_attributes() { set_timestep(m_timestep); }

  void initialize(){};

  void processing() { i.write(PvCell::singleDiodeEquation(1000, v.read())); }

  void ac_processing(){};

  SCA_CTOR(PvCellSupply) {
    m_timestep = sc_core::sc_time::from_seconds(
        Config::get().getDouble("PowerModelTimestep"));

    // TODO Read CSV irradiance trace
  };

private:
  sc_core::sc_time m_timestep; // Evaluation timestep
};

SCA_TDF_MODULE(BoostRegulator) {
  // Consume output and input voltage
  sca_tdf::sca_in<double> v_in;
  sca_tdf::sca_in<double> v_out;

  // Produce output and input current
  sca_tdf::sca_out<double> i_out;
  sca_tdf::sca_out<double> i_in;

  void set_attributes() { set_timestep(m_timestep); }

  void initialize(){};

  void processing() {
    if (v_in.read() > m_inputVoltageLowThreshold &&
        v_out < m_outputVoltageSetPoint) {
      // Calculate power
      double inputPower = m_inputCurrentLimit * v_in.read();
      double outputPower = (inputPower - m_quiescentCurrent) * m_efficiency;
      i_in.write(m_inputCurrentLimit);
      i_out.write(std::min(outputPower / v_out.read(), m_outputCurrentLimit));
      spdlog::info("Starting regulator, outputting {:e} A, drawing {:e} A, "
                   "input voltage {:e}, output voltage {:e} V",
                   std::min(outputPower / v_out.read(), m_outputCurrentLimit),
                   m_inputCurrentLimit, v_in.read(), v_out.read());
    } else {
      // spdlog::info("Waiting for input cap to recover ({:e}v)", v_in.read());
      i_in.write(m_quiescentCurrent);
      i_out.write(0.0);
    }
  }

  void ac_processing(){};

  SCA_CTOR(BoostRegulator) {
    m_timestep = sc_core::sc_time::from_seconds(
        Config::get().getDouble("PowerModelTimestep"));
  };

private:
  double m_inputCurrentLimit = 1000e-6;            // [Ampere]
  double m_outputCurrentLimit = 5e-4;              // [Ampere]
  double m_inputVoltageLowThreshold = 0.8 * 0.461; // [Volt]
  double m_outputVoltageSetPoint = 3.6;            // [Volt]
  double m_quiescentCurrent = 488e-9;              // [Ampere]
  double m_efficiency = 0.8;                       // [Per cent]
  sc_core::sc_time m_timestep;                     // Evaluation timestep
};
