/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "ps/PowerModelChannelIf.hpp"
#include "ps/PowerModelEventBase.hpp"
#include <memory>
#include <string>
#include <systemc>
#include <vector>

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
  virtual int
  registerEvent(const std::string moduleName,
                std::shared_ptr<PowerModelEventBase> eventPtr) override;

  virtual int
  registerState(const std::string moduleName,
                std::shared_ptr<PowerModelStateBase> statePtr) override;

  virtual void reportEvent(const int eventId, const int n = 1) override;

  virtual void reportState(const int stateId) override;

  virtual int popEventCount(const int eventId) override;

  virtual double popEventEnergy(const int eventId) override;

  virtual double popDynamicEnergy() override;

  virtual double getStaticCurrent() override;

  virtual const sc_core::sc_event &supplyVoltageChangedEvent() const override {
    return m_supplyVoltageChangedEvent;
  }

  virtual double getSupplyVoltage() const override { return m_supplyVoltage; }

  virtual void setSupplyVoltage(double val) override;

  /**
   * @brief start_of_simulation systemc callback. Used here to initialize the
   * internal event log.
   */
  virtual void start_of_simulation() override;

private:
  //! Supply voltage associated with this channel
  double m_supplyVoltage = 0.0;

  //! SystemC event
  sc_core::sc_event m_supplyVoltageChangedEvent{"supplyVoltageChangedEvent"};

  //! Vector of module names that use state/event reporting. The  index
  //! corresponds to the module id, and is only used internally.
  std::vector<std::string> m_moduleNames;

  // ------ Events ------
  //! Struct for storing state objects and their module ids
  struct ModuleEventEntry {
    std::shared_ptr<PowerModelEventBase> event;
    const int moduleId;
    ModuleEventEntry(std::shared_ptr<PowerModelEventBase> &&event_,
                     const int id)
        : event(std::move(event_)), moduleId(id) {}
  };

  //! Stores registered events. The index corresponds to the event id
  std::vector<ModuleEventEntry> m_events;

  //! Keeps track of event counts since the last pop
  std::vector<int> m_eventRates;

  // ------ States ------
  //! Struct for storing state objects and their module ids
  struct ModuleStateEntry {
    std::shared_ptr<PowerModelStateBase> state;
    const int moduleId;
    ModuleStateEntry(std::shared_ptr<PowerModelStateBase> &&state_,
                     const int id)
        : state(std::move(state_)), moduleId(id) {}
  };

  //! Vector for storing state objects and their corresponding module ids. The
  //! index of this vector corresponds to state ids.
  std::vector<ModuleStateEntry> m_states;

  //! Current state of modules. Defaults to the first registered state for each
  //! module. The index is the module id and the value is the state id.
  std::vector<int> m_currentStates;

  // ------ Logging ------
  std::string m_eventlogFileName;
  std::string m_staticPowerLogFileName;

  //! Log file timestep
  sc_core::sc_time m_logTimestep;

  //! Keeps log of event counts in the form:
  //! count0 count1 ... countN TIME0(microseconds)
  //! count0 count1 ... countN TIME1(microseconds)
  //! ...
  //! count0 count1 ... countN TIMEM(microseconds)
  std::vector<std::vector<int>> m_log;

  // Keeps log of static power in the form:
  // i_mod0 i_mod1 ... i_modN TIME0
  // i_mod0 i_mod1 ... i_modN TIME1
  // ...
  // i_mod0 i_mod1 ... i_modN TIMEM
  std::vector<std::vector<double>> m_staticPowerLog;

  //! How many log entries to save in memory before dumping to file
  const int m_logDumpThreshold = 100E3;

  //! How many static power log entries to average out
  const int m_staticPowerAveragingFactor = 100;

  /**
   * @brief dumpEventCsv a method that writes the event log to a csv.
   */
  void dumpEventCsv();

  void dumpStaticPowerCsv();

  /**
   * @brief logLoop systemc thread that records event counts at a specified
   * timestep. The event counts for logging are unaffected reset by the
   * channel's reader.
   */
  void logLoop();
};
