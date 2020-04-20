/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string>
#include <tuple>
#include <unordered_map>
#include "ps/PowerCalculator.hpp"
#include "utilities/Config.hpp"

bool PowerCalculator::hasCoeff(const std::string &name) const {
  const auto it = m_coeffs.find(name);
  if (it == m_coeffs.end()) {
    return Config::get().contains(name);
  } else {
    return true;
  }
}

double PowerCalculator::getCoeff(const std::string &name) {
  const auto it = m_coeffs.find(name);
  if (it == m_coeffs.end()) {
    double newCoeff;
    try {
      newCoeff = Config::get().getDouble(name);
    } catch (std::invalid_argument &e) {
      newCoeff = 0.0;
    }
    m_coeffs[name] = newCoeff;
  }
  return m_coeffs[name];
}

double PowerCalculator::getEnergy(const std::string &eventname, double cnt) {
  return getCoeff(eventname) * cnt;
}

double PowerCalculator::getEnergy(const eventRate_t &event) {
  std::string nm;
  double rate;
  std::tie(nm, rate) = event;
  return getCoeff(nm) * rate;
}

double PowerCalculator::getEnergy(const std::vector<eventRate_t> &events) {
  double energy = 0.0;
  for (const auto &e : events) {
    std::string nm;
    double rate;
    std::tie(nm, rate) = e;
    energy += getCoeff(nm) * rate;
  }
  return energy;
}
