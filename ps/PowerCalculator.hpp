/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <tuple>
#include <vector>

class PowerCalculator {
 public:
  /****** Public types ******/
  typedef std::tuple<std::string, double> eventRate_t;
  struct EventRate {
    enum size_t { Name, Value };
  };

  /****** Public functions ******/
  explicit PowerCalculator(std::string fn = "");

  double getEnergy(const std::vector<eventRate_t> &events);

  double getEnergy(const eventRate_t &event);

  double getEnergy(const std::string &eventname, double cnt);

  bool hasCoeff(const std::string &nm);

  double getCoeff(const std::string &nm);

 private:
  /****** Private variables ******/
  YAML::Node m_coeffs;

  /****** Private functions ******/
};
