/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <yaml-cpp/yaml.h>
#include <string>
#include <tuple>
#include "ps/PowerCalculator.hpp"
#include "utilities/Config.hpp"

PowerCalculator::PowerCalculator(std::string fn) {
  if (fn == "") {
    fn = Config::get().getString("EnergyCoeffsPath");
  }
  // Load coefficients from YAML file
  m_coeffs = YAML::LoadFile(fn);
}

double PowerCalculator::getEnergy(const std::vector<eventRate_t> &events) {
  double energy = 0.0;

  for (const auto &e : events) {
    if (m_coeffs[std::get<EventRate::Name>(e)]) {
      energy += m_coeffs[std::get<EventRate::Name>(e)].as<double>() *
                std::get<EventRate::Value>(e);
    }
  }
  return energy;
}

bool PowerCalculator::hasCoeff(const std::string &nm) {
  return (m_coeffs[nm] != NULL);
}

double PowerCalculator::getCoeff(const std::string &nm) {
  return m_coeffs[nm] != NULL ? m_coeffs[nm].as<double>() : 0.0;
}

double PowerCalculator::getEnergy(const std::string &eventname, double cnt) {
  return (m_coeffs[eventname].as<double>() * cnt);
}

double PowerCalculator::getEnergy(const eventRate_t &event) {
  return (m_coeffs[std::get<EventRate::Name>(event)].as<double>() *
          std::get<EventRate::Value>(event));
}
