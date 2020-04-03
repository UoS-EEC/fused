/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <stdint.h>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <systemc>
#include <utility>
#include "ps/EventLog.hpp"
#include "ps/PowerCalculator.hpp"
#include "utilities/Config.hpp"

using namespace sc_core;

EventLog::EventLog(sc_module_name nm) : sc_module(nm) {
  SC_THREAD(process)
  // Load energy cost of events
  m_pcalc = PowerCalculator(Config::get().getString("EnergyCoeffsPath"));
  m_timestep = sc_core::sc_time::from_seconds(
      Config::get().getDouble("EventLogTimeStep"));
}

EventLog::eventId EventLog::registerEvent(std::string name) {
  m_log.emplace_back(column(name, std::vector<uint64_t>(/*n=*/1, /*v=*/0)));
  return m_log.size() - 1;
}

void EventLog::increment(EventLog::eventId id, unsigned int n) {
  if (id >= m_log.size()) {
    SC_REPORT_ERROR(this->name(), "Error invalid event ID.");
  }
  std::get<EVENT_VALUES>(m_log[id]).back() += n;
}

void EventLog::reportState(const std::string &reporter,
                           const std::string &state) {
  auto tmp = std::find_if(m_states.begin(), m_states.end(),
                          [reporter](std::pair<std::string, std::string> &v) {
                            return v.first == reporter;
                          });
  if (tmp == m_states.end()) {
    // New reporter
    m_states.push_back(std::make_pair(reporter, state));
    m_staticConsumption += m_pcalc.getCoeff(reporter + " " + state);
  } else {
    auto &crntState = tmp->second;
    if (crntState != state) {
      m_staticConsumption = m_staticConsumption -
                            m_pcalc.getCoeff(reporter + " " + crntState) +
                            m_pcalc.getCoeff(reporter + " " + state);
      tmp->second = state;
    }
  }
}

void EventLog::dumpCsv(std::string path) {
  if (path == "") {
    path = Config::get().getString("OutputDirectory") + "/eventLog.csv";
  }
  std::ofstream f(path);
  // Header
  for (unsigned int i = 0; i < m_log.size(); i++) {
    f << std::get<EVENT_NAME>(m_log[i])
      << ((i < m_log.size() - 1) ? ',' : '\n');
  }

  // Values
  for (unsigned int i = 0; i < std::get<EVENT_VALUES>(m_log[0]).size(); i++) {
    for (unsigned int j = 0; j < m_log.size(); j++) {
      f << std::get<EVENT_VALUES>(m_log[j])[i]
        << ((j < m_log.size() - 1) ? ',' : '\n');
    }
  }

  f.close();
}

void EventLog::process() {
  // Create event for logging time
  eventId timeId = registerEvent(std::string("time"));

  // Wait for start of simulation, i.e. when all events have been registered
  wait(SC_ZERO_TIME);

  // Construct vector of event ids that have coefficients and their
  // corresponding energy consumption
  std::vector<std::pair<unsigned int, double>> consumerEvents;
  for (unsigned int i = 0; i < m_log.size(); i++) {
    if (m_pcalc.hasCoeff((std::get<EVENT_NAME>(m_log[i])))) {
      consumerEvents.emplace_back(
          std::make_pair(i, m_pcalc.getCoeff(std::get<EVENT_NAME>(m_log[i]))));
    }
  }

  auto triggerConfig = Config::get().getString("EventLogStart");
  if (triggerConfig == "event") {
    spdlog::info("{}: waiting for trigger.", this->name());
    wait(m_startLoggingEvent);  // Start logging at event
  } else if (triggerConfig == "simulation_start") {
    // Start logging at t=0
  } else {
    spdlog::error("{}: Invalid configuration for EventLogStart: {}",
                  this->name(), triggerConfig);
    SC_REPORT_FATAL(this->name(), "Invalid config");
  }

  spdlog::info("{}: Logging started @{:010d} ns", this->name(),
               static_cast<unsigned>(1E9 * sc_time_stamp().to_seconds()));

  // Reset event counters
  for (unsigned int i = 0; i < m_log.size(); i++) {
    if (i != timeId) {
      std::get<EVENT_VALUES>(m_log[i]).back() = 0;  // Reset counters
    } else {
      std::get<EVENT_VALUES>(m_log[i]).back() =
          static_cast<unsigned int>(1E6 * sc_time_stamp().to_seconds());
    }
  }

  while (1) {
    // Wait for timestep
    wait(m_timestep);

    // Consume energy at end of timestep
    staticPower->write(m_staticConsumption);
    double e = std::accumulate(
        consumerEvents.begin(), consumerEvents.end(), 0.0,
        [&](double &a, const std::pair<unsigned int, double> &i) {
          return a + (i.second * std::get<EVENT_VALUES>(m_log[i.first]).back());
        });
    dynamicEnergy->write(e);

    // Create new log entry
    for (auto &v : m_log) {
      std::get<EVENT_VALUES>(v).emplace_back(0);
    }
    std::get<EVENT_VALUES>(m_log[timeId]).back() =
        static_cast<unsigned int>(1E6 * sc_time_stamp().to_seconds());
  }
}
