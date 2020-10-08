/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <systemc>
#include <vector>
#include "libs/strtk.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

template <int S, int D>
class PowerCombine : public sc_core::sc_module {
  SC_HAS_PROCESS(PowerCombine);

 public:
  /* ------ Ports ------ */
  std::array<
      sc_core::sc_port<DynamicEnergyIf, 1, sc_core::SC_ONE_OR_MORE_BOUND>, D>
      dynamicConsumers;
  std::array<sc_core::sc_in<double>, S> staticConsumers;
  sc_core::sc_in<double> vcc;
  sc_core::sc_in<bool> nReset;
  sc_core::sc_out<double> sum;

  PowerCombine(sc_core::sc_module_name nm) {
    // Load constants
    m_timestep = sc_core::sc_time::from_seconds(
        Config::get().getDouble("MCUPowerTimestep"));
    m_vCore = Config::get().getDouble("CpuCoreVoltage");

    // Load vcc current multiplier to compensate for LDO nonlinearity
    auto path = Config::get().getString("VccMultiplierPath");
    if (path != "none") {
      Utility::assertFileExists(Config::get().getString("VccMultiplierPath"));
      std::ifstream file(Config::get().getString("VccMultiplierPath"));
      std::stringstream buffer;
      buffer << file.rdbuf();
      std::string content = buffer.str();
      content.erase(0, content.find("\n") + 1);

      strtk::token_grid grid(content, content.size(), ",");
      for (std::size_t i = 0; i < grid.row_count(); i++) {
        m_vccCurrentMultiplier.push_back(grid.row(i).get<double>(2));
      }
      m_vccCurrentMultiplierResolution =
          grid.row(1).get<double>(0) - grid.row(0).get<double>(0);
      m_vccCurrentMultiplierOffset = grid.row(0).get<double>(0);
      m_compensateLdo = true;
    }

    // Set up thread
    SC_THREAD(process);
  }

 private:
  /* ------ Private variables ------ */
  bool m_compensateLdo{false};
  double m_vCore;  //! Core voltage
  sc_core::sc_time m_timestep;
  std::vector<double> m_vccCurrentMultiplier;
  double m_vccCurrentMultiplierResolution;
  double m_vccCurrentMultiplierOffset;

  /* ------ Private functions ------ */
  void process() {
    wait(sc_core::SC_ZERO_TIME);

    while (1) {
      // Total dynamic energy -> current
      double tot = 0.0;
      for (int i = 0; i < D; i++) {
        // Energy is consumed @ core voltage
        tot +=
            dynamicConsumers[i]->read() / (m_vCore * m_timestep.to_seconds());
      }

      // Total static current
      for (int i = 0; i < S; i++) {
        tot += staticConsumers[i]->read();
      }
      if (m_compensateLdo) {
        // Multiply by vcc-dependent current multiplier
        unsigned idx = nReset.read()
                           ? static_cast<unsigned int>(round(
                                 (vcc.read() - m_vccCurrentMultiplierOffset) /
                                 m_vccCurrentMultiplierResolution))
                           : 0;
        if (idx >= m_vccCurrentMultiplier.size()) {
          idx = m_vccCurrentMultiplier.size() - 1;
        } else if (idx < 0) {
          idx = 0;
        }
        tot *= m_vccCurrentMultiplier[idx];
      }
      sum.write(tot);
      wait(m_timestep);
    }
  }
};
