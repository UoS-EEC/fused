/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <systemc>
#include <tuple>
#include <utility>
#include <vector>
#include "ps/DynamicEnergyIf.hpp"
#include "ps/PowerCalculator.hpp"

class EventLog : public sc_core::sc_module {
 public:
  SC_HAS_PROCESS(EventLog);

  /* ------ Ports ------ */
  sc_core::sc_port<DynamicEnergyIf, 1, sc_core::SC_ONE_OR_MORE_BOUND>
      dynamicEnergy{"dynamicEnergy"};
  sc_core::sc_out<double> staticPower{"staticPower"};

  /* ------ Public methods ------ */
  static EventLog &getInstance() {
    static EventLog instance;
    return instance;
  }

  typedef unsigned int eventId;
  typedef unsigned int stateId;

  /**
   * @brief registerEvent register a new event
   * @param name name of event
   * @return  assigned event id
   */
  eventId registerEvent(std::string name);

  /**
   * @brief reportState Report the state of a module
   * @param reporter module name
   * @param state state descriptor of reporter (module).
   */
  void reportState(const std::string &reporter, const std::string &state);

  /**
   * @brief increment increment event counter by n
   * @param id event id
   * @param n amount to increment by
   */
  void increment(EventLog::eventId id, unsigned int n = 1);

  /**
   * @brief dumpCsv dump eventlog to csv
   * @param path
   */
  void dumpCsv(std::string path = "");

  /**
   * @brief getStartLoggingEvent get the event that triggers the start of event
   * logging.
   * @retval sc_event startLoggingEvent
   */
  sc_core::sc_event &getStartLoggingEvent() { return m_startLoggingEvent; };

  /**
   * @brief startLogging start logging after delay
   * @note set delay to sc_core::SC_ZERO_TIME to start log immediately.
   * @param delay how long to delay before starting the logging.
   */
  void startLogging(const sc_core::sc_time &delay) {
    m_startLoggingEvent.notify(delay);
  }

 private:
  /**
   * @brief EventLog Singleton constructor
   * @param nm
   */
  EventLog(sc_core::sc_module_name nm = "eventLog");

  /**
   * @brief process log events
   */
  [[noreturn]] void process();

 public:
 private:
  /* ------ Private variables ------ */
  std::vector<std::pair<std::string, std::string>> m_states;
  typedef std::tuple<std::string, std::vector<uint64_t>> column;
  enum logIndex { EVENT_NAME, EVENT_VALUES };
  PowerCalculator m_pcalc;
  double m_staticConsumption;

  sc_core::sc_event m_startLoggingEvent{"startLoggingEvent"};
  sc_core::sc_time m_timestep;
  std::vector<column> m_log{};
};
