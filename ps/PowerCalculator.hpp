/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

class PowerCalculator {
 public:
  /****** Public types ******/
  typedef std::tuple<std::string, double> eventRate_t;
  struct EventRate {
    enum size_t { Name, Value };
  };

  /****** Public functions ******/

  /**
   * @brief getEnergy Calculate energy consumption based on event rate and event
   * coefficient.
   * @param eventname
   * @param cnt number of occurrences of event
   * @retval energy consumption
   */
  double getEnergy(const std::string &eventname, double cnt);

  /**
   * @brief getEnergy Calculate energy consumption based on event rate and event
   * coefficient.
   * @param event
   * @retval energy consumption
   */
  double getEnergy(const eventRate_t &event);

  /**
   * @brief getEnergy Calculate energy consumption based on event rate and event
   * coefficient.
   * @param events vector of events
   * @retval energy consumption
   */
  double getEnergy(const std::vector<eventRate_t> &events);

  /**
   * @brief hasCoeff check whether a power/energy coefficient for the
   * specified name is present.
   * @param name Name
   * @retval true if there is an coefficient for the name, false otherwise.
   */
  bool hasCoeff(const std::string &name) const;

  /**
   * @brief getCoeff get the power/energy coefficient for the specified name.
   * @param name Name
   * @retval true if there is an coefficient for name, false otherwise.
   */
  double getCoeff(const std::string &name);

 private:
  /****** Private variables ******/
  std::unordered_map<std::string, double> m_coeffs;  //! Local copy of coeffs

  /****** Private functions ******/
};
