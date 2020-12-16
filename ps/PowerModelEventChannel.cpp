/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <algorithm>
#include <memory>
#include <systemc>
#include <vector>
#include "ps/PowerModelEventBase.hpp"
#include "ps/PowerModelEventChannel.hpp"

using namespace sc_core;

int PowerModelEventChannel::registerEvent(
    std::unique_ptr<PowerModelEventBase> eventPtr) {
  eventPtr->id = m_events.size();
  // TODO check if event name already exists in m_events
  m_events.push_back(std::move(eventPtr));
  m_eventRates.push_back(0);
  sc_assert(m_events.size() == m_eventRates.size());
  return m_events.size() - 1;
}

void PowerModelEventChannel::write(const int eventId, const int n) {
  m_eventRates[eventId] += n;
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
    popEnergy(i, supplyVoltage);
  }
  return result;
}

size_t PowerModelEventChannel::size() const { return m_events.size(); }
