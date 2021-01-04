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
#include "ps/PowerModelChannelIf.hpp"
#include "ps/PowerModelEventBase.hpp"

/**
 * class PowerModelChannel implementation of power model channel.  See
 * interface PowerModelChannelIf.hpp for description.
 *
 * Logging: This implementation optionally writes a csv-formatted log of event
 * rates and module states at a specified time step.
 */
class PowerModelChannel : public virtual PowerModelChannelOutIf,
                          public virtual PowerModelChannelInIf,
                          public sc_core::sc_module {
 public:
  /* ------ Public methods ------ */

  //! Constructor
  PowerModelChannel(const sc_core::sc_module_name name,
                    const std::string logfile = "",
                    const sc_core::sc_time logTimestep = sc_core::SC_ZERO_TIME);

  //! Destructor
  ~PowerModelChannel();

  // See PowerModelChannelIf.hpp for description of the following virtual
  // methods.
  virtual int registerEvent(
      std::unique_ptr<PowerModelEventBase> eventPtr) override;

  virtual int registerState(
      const std::string moduleName,
      std::unique_ptr<PowerModelStateBase> statePtr) override;

  virtual void reportEvent(const int eventId, const int n = 1) override;

  virtual void reportState(const int stateId) override;

  virtual int popEventCount(const int eventId) override;

  virtual size_t size() const override;

  virtual double popEventEnergy(const int eventId,
                                double supplyVoltage) override;

  virtual double popDynamicEnergy(double supplyVoltage) override;

  virtual double getStaticCurrent(double supplyVoltage,
                                  double clockFrequency) override;

  /**
   * @brief start_of_simulation systemc callback. Used here to initialize the
   * internal event log.
   */
  virtual void start_of_simulation() override;

 private:
  // ------ Events ------
  //! Stores registered events. The index corresponds to the event id
  std::vector<std::unique_ptr<PowerModelEventBase>> m_events;

  //! Keeps track of event counts since the last pop
  std::vector<int> m_eventRates;

  // ------ States ------
  //! Struct for storing state objects and their module ids
  struct ModuleStateEntry {
    std::unique_ptr<PowerModelStateBase> state;
    const int moduleId;
    ModuleStateEntry(std::unique_ptr<PowerModelStateBase>&& state_,
                     const int id)
        : state(std::move(state_)), moduleId(id) {}
  };

  //! Vector for storing state objects and their corresponding module ids. The
  //! index of this vector corresponds to state ids.
  std::vector<ModuleStateEntry> m_states;

  //! Vector of module names that use state reporting. The  index corresponds to
  //! the module id, and is only used internally.
  std::vector<std::string> m_moduleNames;

  //! Current state of modules. Defaults to the first registered state for each
  //! module. The index is the module id and the value is the state id.
  std::vector<int> m_currentStates;

  // ------ Logging ------
  //! Log file path
  std::string m_logFilePath;

  //! Log file timestep
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
   * @brief dumpEventCsv a method that writes the event log to a csv.
   */
  void dumpEventCsv();

  /**
   * @brief logLoop systemc thread that records event counts at a specified
   * timestep. The event counts for logging are unaffected reset by the
   * channel's reader.
   */
  void logLoop();
};
