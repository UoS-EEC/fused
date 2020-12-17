/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <systemc>
#include <vector>
#include "ps/PowerModelEventBase.hpp"
#include "ps/PowerModelEventChannel.hpp"

using namespace sc_core;

PowerModelEventChannel::PowerModelEventChannel(const sc_module_name name,
                                               const std::string logFileName,
                                               sc_time logTimestep)
    : sc_module(name), m_logFileName(logFileName), m_logTimestep(logTimestep) {
  // Create/overwrite log file
  std::ofstream f(logFileName);
  if (!f.good()) {
    SC_REPORT_FATAL(
        this->name(),
        fmt::format("Can't open eventLog file at {}", logFileName).c_str());
  }
}

PowerModelEventChannel::~PowerModelEventChannel() { dumpCsv(); }

int PowerModelEventChannel::registerEvent(
    std::unique_ptr<PowerModelEventBase> eventPtr) {
  SC_HAS_PROCESS(PowerModelEventChannel);
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
        FMT_STRING("PowerModelEventChannel::registerEvent event name {:s} "
                   "already registered"),
        name));
  }

  eventPtr->id = m_events.size();  // Set ID
  m_events.push_back(std::move(eventPtr));
  m_eventRates.push_back(0);
  sc_assert(m_events.size() == m_eventRates.size());
  return m_events.back()->id;
}

void PowerModelEventChannel::write(const int eventId, const int n) {
  m_eventRates[eventId] += n;
  m_log.back()[eventId] += n;
}

int PowerModelEventChannel::pop(const int eventId) {
  const auto tmp = m_eventRates[eventId];
  m_eventRates[eventId] = 0;
  return tmp;
}

double PowerModelEventChannel::popEnergy(const int eventId,
                                         const double supplyVoltage) {
  return m_events[eventId]->calculateEnergy(supplyVoltage) * pop(eventId);
}

double PowerModelEventChannel::popEnergy(const double supplyVoltage) {
  double result = 0.0;
  for (int i = 0; i < m_events.size(); ++i) {
    result += popEnergy(i, supplyVoltage);
  }
  return result;
}

size_t PowerModelEventChannel::size() const { return m_events.size(); }

void PowerModelEventChannel::start_of_simulation() {
  // Initialize event log
  m_log.push_back(std::vector<int>(m_events.size() + 1, 0));
  // Fist entry is at t = timestep
  m_log.back().back() = static_cast<int>(m_logTimestep.to_seconds() * 1.0e6);
}

void PowerModelEventChannel::logLoop() {
  if (m_logFileName == "" || m_logTimestep == SC_ZERO_TIME) {
    SC_REPORT_INFO(this->name(), "Logging disabled.");
    return;
  }

  while (1) {
    // Wait for a timestep
    wait(m_logTimestep);

    // Push new timestep
    m_log.push_back(std::vector<int>(m_events.size() + 1, 0));
    m_log.back().back() = static_cast<int>(
        (m_logTimestep + sc_time_stamp()).to_seconds() * 1.0e6);

    // Dump file when log exceeds threshold
    if (m_log.size() > m_logDumpThreshold) {
      dumpCsv();
      m_log.clear();
    }
  }
}

void PowerModelEventChannel::dumpCsv() {
  std::ofstream f(m_logFileName, std::ios::out | std::ios::app);
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
