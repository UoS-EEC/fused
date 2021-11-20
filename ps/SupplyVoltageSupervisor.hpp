/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include <spdlog/spdlog.h>
#include <systemc-ams>
#include <systemc>

/*
 * Description:
 *
 * A simple supply voltage supervisor module that uses a voltage detector and
 * a load switch to:
 *   - Enable/connect the output voltage when the input voltage exceeds an
 *     on-threshold OR the forceOn signal is asserted.
 *   - Disable/disconnect the output voltage when the input voltage is below an
 *     off-threshold.
 *   - Assert a v_warn (voltage warning) signal when the input voltage is below
 *     a voltage warning threshold.
 *
 */

// Load switch with voltage detector and override input.
// Consumes ext.dc uA  internally
SC_MODULE(SupplyVoltageSupervisor) {
  // Ports
  sca_tdf::sca_in<bool> force{"force"};
  sca_tdf::sca_in<double> i_out{"i_out"};
  sca_tdf::sca_in<double> v_in{"v_in"};
  sca_tdf::sca_out<double> v_out{"v_out"};
  sca_tdf::sca_out<double> i_in{"i_in"};
  sca_tdf::sca_out<sc_dt::sc_logic> warn{"warn"};

  SCA_TDF_MODULE(VoltageDetectorWithOverride) {
    sca_tdf::sca_in<double> v{"v"};
    sca_tdf::sca_in<bool> force{"force"};
    sca_tdf::sca_out<bool> out{"v_out"};
    sca_tdf::sca_out<sc_dt::sc_logic> warn{"warn"};

    VoltageDetectorWithOverride(const sc_core::sc_module_name name,
                                const double vOn, const double vOff,
                                const double vWarn)
        : sca_tdf::sca_module(name), m_vOn(vOn), m_vOff(vOff), m_vWarn(vWarn) {}

    void processing() {
      const auto currentVin = v.read();
      m_isOn = force.read() || (currentVin > m_vOn) ||
               ((currentVin > m_vOff) && m_isOn);

      out.write(m_isOn);

      // Print voltage when supply switched off
      if (!m_isOn && m_wasOn) {
        spdlog::info("{:s}: @{:.0f} ns output turned off at v_cap={:.3f}",
                     this->name(),
                     sc_core::sc_time_stamp().to_seconds() * 1.0e9, currentVin);
      }
      m_wasOn = m_isOn;
      /*
      spdlog::info("{:s}: @{:.0f} ns enable={:d} at v_cap={:.3f}", this->name(),
                   sc_core::sc_time_stamp().to_seconds() * 1.0e9, m_isOn,
                   currentVin);
                   */

      // Issue voltage warning
      warn.write(sc_dt::sc_logic(currentVin < m_vWarn));
    }

  private:
    const double m_vOn;
    const double m_vOff;
    const double m_vWarn;
    bool m_wasOn{false};
    bool m_isOn{false};
  };

  SCA_TDF_MODULE(Switch) {
    sca_tdf::sca_in<bool> ctrl{"ctrl"};
    sca_tdf::sca_in<double> in{"in"};
    sca_tdf::sca_out<double> out{"out"};

    SCA_CTOR(Switch) {}

    void processing() {
      if (ctrl.read()) {
        out.write(in.read());
      } else {
        out.write(0.0);
      }
    }
  };

  SupplyVoltageSupervisor(const sc_core::sc_module_name name, const double vOn,
                          const double vOff, const double vWarn)
      : sc_core::sc_module(name), vdet("vdet", vOn, vOff, vWarn) {
    // Voltage detector
    vdet.v(v_in);
    vdet.force(force);
    vdet.out(outputOn);
    vdet.warn(warn);

    /**
     * Note:
     * The voltage and current are switched separately to enable TDF scheduling
     * them separately, e.g. when there's a loop from consumed current to
     * produced voltage.
     */

    // Voltage switch
    voltageSwitch.ctrl(outputOn);
    voltageSwitch.in(v_in);
    voltageSwitch.out(v_out);

    // Current switch
    currentSwitch.ctrl(outputOn);
    currentSwitch.in(i_out); // (current flows in the opposite direction)
    currentSwitch.out(i_in);
  }

private:
  // Submodules
  VoltageDetectorWithOverride vdet;
  Switch currentSwitch{"currentSwitch"};
  Switch voltageSwitch{"voltageSwitch"};

  // Signals
  sca_tdf::sca_signal<bool> outputOn{"outputOn"};
};
