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

  void processing() {
    int luxIdx =
        int(sc_core::sc_time_stamp() / m_luxTraceSampleTime) % luxTrace.size();
    double iOut;

    if (luxTrace[luxIdx] > 100) {
      iOut = PvCell::singleDiodeEquation(luxTrace[luxIdx], v.read());
    } else {
      // No point in calculating the PV cell current when there's no light
      iOut = 0.0;
    }

    i.write(iOut);

    // Write luxtrace into signal for tracing
    irradiance.write(luxTrace[luxIdx]);
    powerOut.write(iOut * v.read());
  }

  void ac_processing(){};

  SCA_CTOR(PvCellSupply) {
    m_timestep = sc_core::sc_time::from_seconds(
        Config::get().getDouble("PowerModelTimestep"));

    // Load irradiance input trace
    const bool validTraceFile =
        Config::get().contains("LuxTraceSampleTime") &&
        (Config::get().contains("LuxTracePath")
             ? Config::get().getString("LuxTracePath") != "none"
             : false);

    if (validTraceFile) {
      auto fn = Config::get().getString("LuxTracePath");
      m_luxTraceSampleTime = sc_core::sc_time::from_seconds(
          Config::get().getDouble("LuxTraceSampleTime"));
      Utility::assertFileExists(fn);
      std::ifstream infile(fn);

      if (infile.bad()) {
        SC_REPORT_FATAL(
            this->name(),
            fmt::format("Couldn't open trace file {:s}", fn).c_str());
      }

      int tmp;
      infile >> tmp;
      luxTrace.push_back(tmp);
      while (!infile.eof()) { // keep reading until end-of-file
        infile >> tmp;        // sets EOF flag if no value found
        luxTrace.push_back(tmp);
      }
      spdlog::info("{:s}: successfully loaded input trace with {:d} datapoints "
                   "from {:s}",
                   this->name(), luxTrace.size(), fn);
    } else {
      luxTrace.push_back(0);
      spdlog::info("{:s}: No lux trace specified, using a static value of {:d}",
                   this->name(), luxTrace[0]);
      m_luxTraceSampleTime = sc_core::sc_time(1, sc_core::SC_MS);
    }
  };

private:
  sc_core::sc_time m_timestep;           // Evaluation timestep
  sc_core::sc_time m_luxTraceSampleTime; // Evaluation timestep

  //! Timer-series trace of irradiance, one value per second. Unit: Lux
  std::vector<int> luxTrace;

public:
  // Signals for tracing
  sc_core::sc_signal<int> irradiance;  // Signal used for tracing
  sc_core::sc_signal<double> powerOut; // Signal used for tracing
};

SCA_TDF_MODULE(BoostRegulator) {
  // Consume output and input voltage
  sca_tdf::sca_in<double> v_in;
  sca_tdf::sca_in<double> v_out;

  // Produce output and input current
  sca_tdf::sca_out<double> i_out;
  sca_tdf::sca_out<double> i_in;
  sca_tdf::sca_out<bool> output_ok;

  void set_attributes() { set_timestep(m_timestep); }

  void initialize(){};

  void processing() {

    if (v_in.read() > m_inputVoltageOkThreshold && !m_isOn) {
      spdlog::info("{:s}: @{:.0f} ns Starting regulator", this->name(),
                   sc_core::sc_time_stamp().to_seconds() * 1.0e9);
      m_isOn = true;
    } else if (v_in.read() < m_inputVoltageLowThreshold && m_isOn) {
      spdlog::info("{:s}: @{:.0f} ns Stopping regulator", this->name(),
                   sc_core::sc_time_stamp().to_seconds() * 1.0e9);
      m_isOn = false;
    }

    if (m_isOn && v_out < m_outputVoltageSetPoint) {
      // Calculate power
      double inputPower = m_inputCurrentLimit * v_in.read();
      double outputPower = (inputPower - m_quiescentCurrent) * m_efficiency;
      i_in.write(m_inputCurrentLimit);
      i_out.write(std::min(outputPower / v_out.read(), m_outputCurrentLimit));

      spdlog::info("{:s}: outputting {:e} A, drawing {:e} A, input voltage "
                   "{:e}, output voltage {:e} V",
                   this->name(),
                   std::min(outputPower / v_out.read(), m_outputCurrentLimit),
                   m_inputCurrentLimit, v_in.read(), v_out.read());
    } else {
      // spdlog::info("Waiting for input cap to recover ({:e}v)",
      // v_in.read());
      if (v_in.read() > 0) {
        i_in.write(m_quiescentCurrent);
      }
      i_out.write(0.0);
    }

    if (v_out.read() >= m_outputVoltageSetPoint) {
      output_ok.write(true);
    } else if (v_out.read() < 0.98 * m_outputVoltageSetPoint) {
      output_ok.write(false);
    }
  }

  void ac_processing(){};

  SCA_CTOR(BoostRegulator) {
    m_timestep = sc_core::sc_time::from_seconds(
        Config::get().getDouble("PowerModelTimestep"));
  };

private:
  bool m_isOn{false};                            // On/off state
  double m_inputCurrentLimit = 1.0e-3;           // [Ampere]
  double m_outputCurrentLimit = 1e-3;            // [Ampere]
  double m_inputVoltageOkThreshold = 0.85 * 1.6; // [Volt]
  double m_inputVoltageLowThreshold = 0.2;       // [Volt]
  double m_outputVoltageSetPoint = 1.85;         // [Volt]
  double m_quiescentCurrent = 488e-9;            // [Ampere]
  double m_efficiency = 0.8;                     // [Per cent]
  sc_core::sc_time m_timestep;                   // Evaluation timestep
};
