/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <systemc>
#include <vector>
#include "ps/PowerModelChannel.hpp"
#include "ps/PowerModelEventBase.hpp"

using namespace sc_core;

PowerModelChannel::PowerModelChannel(const sc_module_name name,
                                     const std::string logFilePath,
                                     sc_time logTimestep)
    : sc_module(name), m_logFilePath(logFilePath), m_logTimestep(logTimestep) {
  if (logFilePath != "none") {
    // Create/overwrite log files
    std::ofstream f(logFilePath + "/eventLog.csv",
                    std::ios::out | std::ios::trunc);
    if (!f.good()) {
      SC_REPORT_FATAL(this->name(),
                      fmt::format("Can't open eventLog file at {}/eventLog.csv",
                                  logFilePath)
                          .c_str());
    }
  }
  SC_HAS_PROCESS(PowerModelChannel);
  SC_THREAD(logLoop);
}

PowerModelChannel::~PowerModelChannel() { dumpEventCsv(); }

int PowerModelChannel::registerEvent(
    std::unique_ptr<PowerModelEventBase> eventPtr) {
  // Check if already running
  if (sc_is_running()) {
    throw std::runtime_error(
        "PowerModelEvent::registerEvent events can not be registered after "
        "simulation has started. Events shall only be registered during "
        "construction/elaboration.");
  }

  // Check if event name already present in m_events
  const auto name = eventPtr->name;
  if (std::any_of(m_events.begin(), m_events.end(),
                  [name](const std::unique_ptr<PowerModelEventBase> &e) {
                    return e->name == name;
                  })) {
    throw std::invalid_argument(fmt::format(
        FMT_STRING("PowerModelChannel::registerEvent event name '{:s}' "
                   "already registered"),
        name));
  }

  eventPtr->id = m_events.size();  // Set ID
  m_events.push_back(std::move(eventPtr));
  m_eventRates.push_back(0);
  sc_assert(m_events.size() == m_eventRates.size());
  return m_events.back()->id;
}

int PowerModelChannel::registerState(
    const std::string moduleName,
    std::unique_ptr<PowerModelStateBase> statePtr) {
  // Check if already running
  if (sc_is_running()) {
    throw std::runtime_error(
        "PowerModelEvent::registerState states can not be registered after "
        "simulation has started. States shall only be registered during "
        "construction/elaboration.");
  }

  const auto it =
      std::find(m_moduleNames.begin(), m_moduleNames.end(), moduleName);
  int moduleId = -1;
  if (it == m_moduleNames.end()) {
    // This is the first state registration for this module
    moduleId = m_moduleNames.size();
    m_moduleNames.push_back(moduleName);
    m_currentStates.push_back(m_states.size());
  } else {
    // This is *not* the first state registration for this module
    // Check if state name already registered for the specified module name
    const auto name = statePtr->name;
    moduleId = it - m_moduleNames.begin();
    if (std::any_of(m_states.begin(), m_states.end(),
                    [name, moduleId](const ModuleStateEntry &s) {
                      return s.state->name == name && s.moduleId == moduleId;
                    })) {
      throw std::invalid_argument(fmt::format(
          FMT_STRING("PowerModelChannel::registerState state '{:s}' already "
                     "registered with module '{:s}'"),
          name, moduleName));
    }
  }

  // Add state to m_states
  statePtr->id = m_states.size();
  m_states.emplace_back(std::move(statePtr), moduleId);

  return m_states.back().state->id;
}

void PowerModelChannel::reportEvent(const int eventId, const int n) {
  if (!sc_is_running()) {
    throw std::runtime_error(
        "PowerModelEvent::reportEvent events can not be reported before"
        "simulation has started. Events shall only be reported during "
        "simulation");
  }
  sc_assert(eventId >= 0 && eventId < m_log.back().size());

  m_eventRates[eventId] += n;
  m_log.back()[eventId] += n;
}

void PowerModelChannel::reportState(const int stateId) {
  if (!sc_is_running()) {
    throw std::runtime_error(
        "PowerModelState::reportState states can not be reported before"
        "simulation has started. States shall only be reported during "
        "simulation");
  }
  sc_assert(stateId >= 0 && stateId < m_states.size());
  const auto mid = m_states[stateId].moduleId;
  m_currentStates[mid] = stateId;
}

int PowerModelChannel::popEventCount(const int eventId) {
  sc_assert(eventId >= 0 && eventId < m_log.back().size());
  const auto tmp = m_eventRates[eventId];
  m_eventRates[eventId] = 0;
  return tmp;
}

double PowerModelChannel::popEventEnergy(const int eventId,
                                         const double supplyVoltage) {
  sc_assert(eventId >= 0 && eventId < m_log.back().size());
  return m_events[eventId]->calculateEnergy(supplyVoltage) *
         popEventCount(eventId);
}

double PowerModelChannel::popDynamicEnergy(const double supplyVoltage) {
  double result = 0.0;
  for (int i = 0; i < m_events.size(); ++i) {
    result += popEventEnergy(i, supplyVoltage);
  }
  return result;
}

double PowerModelChannel::getStaticCurrent(const double supplyVoltage,
                                           const double clockFrequency) {
  return std::accumulate(m_currentStates.begin(), m_currentStates.end(), 0.0,
                         [=](const double &sum, const int &stateId) {
                           return sum +
                                  m_states[stateId].state->calculateCurrent(
                                      supplyVoltage, clockFrequency);
                         });
}

size_t PowerModelChannel::size() const { return m_events.size(); }

void PowerModelChannel::start_of_simulation() {
  // Initialize event log
  m_log.emplace_back(m_events.size() + 1, 0);
  // Fist entry is at t = timestep
  m_log.back().back() = static_cast<int>(m_logTimestep.to_seconds() * 1.0e6);
}

void PowerModelChannel::logLoop() {
  if (m_logFilePath == "none" || m_logTimestep == SC_ZERO_TIME) {
    SC_REPORT_INFO(this->name(), "Logging disabled.");
    return;
  }

  while (1) {
    // Wait for a timestep
    wait(m_logTimestep);

    // Push new timestep
    spdlog::info(FMT_STRING("PowerModelChannel:: Pushing new timestep, "
                            "m_events.size={:d}, m_log.size={:d}"),
                 m_events.size(), m_log.size());
    m_log.emplace_back(m_events.size() + 1, 0);  // New row of all 0s
    // Last column in the new row is the current time step
    m_log.back().back() = static_cast<int>(
        (m_logTimestep + sc_time_stamp()).to_seconds() * 1.0e6);

    // Dump file when log exceeds threshold
    if (m_log.size() > m_logDumpThreshold) {
      dumpEventCsv();
      m_log.clear();
    }
  }
}

void PowerModelChannel::dumpEventCsv() {
  std::ofstream f(m_logFilePath + "eventLog.csv",
                  std::ios::out | std::ios::app);
  if (f.tellp() == 0) {
    // Header
    for (unsigned int i = 0; i < m_events.size() + 1; ++i) {
      if (i < m_events.size()) {
        f << m_events[i]->name << ',';
      } else {
        f << "time(us)\n";
      }
    }
  }

  // Values
  for (const auto &row : m_log) {
    for (const auto &val : row) {
      f << val << (&val == &row.back() ? '\n' : ',');
    }
  }
}
