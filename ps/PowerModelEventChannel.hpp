/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <string>
#include <systemc>
#include <vector>
#include "ps/PowerModelEventBase.hpp"
#include "ps/PowerModelEventChannelIf.hpp"

/**
 * class PowerModelEventChannel implementation of event energy channel.  See
 * interface PowerModelEventChannelIf.hpp for description.
 *
 * Logging: This implementation optionally writes a csv-formatted log of event
 * rates at a specified time step.
 */
class PowerModelEventChannel : public virtual PowerModelEventChannelOutIf,
                               public virtual PowerModelEventChannelInIf,
                               public sc_core::sc_module {
 public:
  /* ------ Public methods ------ */

  //! Constructor
  PowerModelEventChannel(
      const sc_core::sc_module_name name, const std::string logfile = "",
      const sc_core::sc_time logTimestep = sc_core::SC_ZERO_TIME);

  //! Destructor
  ~PowerModelEventChannel();

  // See PowerModelEventChannelIf.hpp for description of the following virtual
  // methods.
  virtual int registerEvent(
      std::unique_ptr<PowerModelEventBase> eventPtr) override;

  virtual void write(const int eventId, const int n = 1) override;

  virtual int pop(const int eventId) override;

  virtual size_t size() const override;

  virtual double popEnergy(const int eventId, double supplyVoltage) override;

  virtual double popEnergy(double supplyVoltage) override;

  /**
   * @brief start_of_simulation systemc callback. Used here to initialize the
   * internal event log.
   */
  virtual void start_of_simulation() override;

 private:
  //! Stores registered events. The index corresponds to the event id
  std::vector<std::unique_ptr<PowerModelEventBase>> m_events;

  //! Keeps track of event counts since the last pop
  std::vector<int> m_eventRates;

  // ------ Logging ------

  //! Event log file
  std::string m_logFileName;

  //! Event log timestep
  sc_core::sc_time m_logTimestep;

  //! Keeps log of event counts in the form:
  //! count0 count1 ... countN TIME0(microseconds)
  //! count0 count1 ... countN TIME1(microseconds)
  //! ...
  //! count0 count1 ... countN TIMEM(microseconds)
  std::vector<std::vector<int>> m_log;

  //! How many log entries to save in memory before dumping to file
  const int m_logDumpThreshold = 100E3;

  /**
   * @brief dumpCsv a method that writes the event log to a csv.
   */
  void dumpCsv();

  /**
   * @brief logLoop systemc thread that records event counts at a specified
   * timestep. The event counts are not reset by pop's by the channel's reader.
   */
  void logLoop();
};
