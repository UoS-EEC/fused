/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include <systemc-ams>
#include <systemc>

/**
 * Linear voltage regulator
 *
 *   A simple model of a linear voltage regulator (aka LDO) with a constant
 *   voltage drop and leakage current.
 *
 *   The current path and voltage path are split into separate SCA modules to
 *   enable easier scheduling.
 *
 *
 *   Takes output current and input voltage as SCA_TDF inputs.
 *   Produces input current and output voltage as SCA_TDF outputs.
 *
 */

SC_MODULE(LinearRegulator) {
  // Consume input voltage and output current
  sca_tdf::sca_in<double> i_out{"i_out"};
  sca_tdf::sca_in<double> v_in{"v_in"};

  // Produce output voltage and input current
  sca_tdf::sca_out<double> v_out{"v_out"};
  sca_tdf::sca_out<double> i_in{"i_in"};

  // ------ Internal  submodules ------
  SCA_TDF_MODULE(LdoCurrentPath) {
    sca_tdf::sca_in<double> i_out{"i_out"};
    sca_tdf::sca_out<double> i_in{"i_in"};

    SCA_CTOR(LdoCurrentPath) {}
    LdoCurrentPath(const sc_core::sc_module_name name, const double leakage)
        : sca_tdf::sca_module(name), m_leakage{leakage} {}

    void processing() { i_in.write(i_out.read() + m_leakage); }

  private:
    double m_leakage;
  };

  SCA_TDF_MODULE(LdoVoltagePath) {
    sca_tdf::sca_out<double> v_out{"v_out"};
    sca_tdf::sca_in<double> v_in{"v_in"};

    // Constructor
    LdoVoltagePath(
        const sc_core::sc_module_name name, const double outputVoltage,
        const double dropoutVoltage = 0.0, const double leakage = 0.0)
        : sca_tdf::sca_module(name), m_outputVoltage(outputVoltage),
          m_dropoutVoltage(dropoutVoltage), m_leakage(leakage) {}

    void processing() {
      double crntVin = v_in.read();

      // Convert voltage
      if (crntVin > m_outputVoltage + m_dropoutVoltage) {
        // Normal operation
        v_out.write(m_outputVoltage);
      } else if (crntVin > m_dropoutVoltage) {
        // Input voltage too low
        v_out.write(crntVin - m_dropoutVoltage);
      } else {
        // Off
        v_out.write(0.0);
      }
    }

  private:
    double m_outputVoltage;
    double m_dropoutVoltage;
    double m_leakage;
  };

  // Constructor
  LinearRegulator(const sc_core::sc_module_name name, double outputVoltage,
                  double dropoutVoltage = 0.0, double leakage = 0.0)
      : sc_core::sc_module(name),
        voltagePath("voltage", outputVoltage, dropoutVoltage, leakage),
        currentPath("current", leakage) {
    voltagePath.v_in(v_in);
    voltagePath.v_out(v_out);

    currentPath.i_out(i_out);
    currentPath.i_in(i_in);
  }

  LdoVoltagePath voltagePath;
  LdoCurrentPath currentPath;
};
