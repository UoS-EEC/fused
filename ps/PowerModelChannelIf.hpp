/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <systemc>
#include "ps/PowerModelEventBase.hpp"
#include "ps/PowerModelStateBase.hpp"

/**
 * Power model channel interfaces
 * ------------------------------------
 *
 *  The event- and state-based power model is implemented in the channel derived
 * from this base class. Modules register their power modelling events and
 * states, and then report event occurrences and current states via a port of
 * type PowerModelOutPort. The cumulative event count/energy and the state
 * current consumption is then available via the PowerModelChannelInIf for power
 * modelling modules.
 *
 */

/**
 * @brief class PowerModelChannelOutIf output interface. This is used
 * by modules to register and report their events and states for power
 * modelling.
 */
class PowerModelChannelOutIf : public virtual sc_core::sc_interface {
 public:
  /**
   * @brief registerEvent register a new power model event. All events must be
   * registered before simulation starts. Trying to register an event during
   * simulation will result in an exception. The returned event id is used for
   * incrementing the count of occurrences of the specified event via
   * reportEvent.
   * @param eventPtr unique pointer to an event derived from PowerModelEventBase
   * @retval  assigned event id
   */
  virtual int registerEvent(std::unique_ptr<PowerModelEventBase> eventPtr) = 0;

  /**
   * @brief registerState register a new power model state. All states must be
   * registered before simulation starts. Trying to register a state during
   * simulation will result in an exception. The returned state id is used for
   * reporting via reportState
   * @param statePtr unique pointer to a state derived from PowerModelStateBase
   * @retval  assigned state id
   */
  virtual int registerState(const std::string moduleName,
                            std::unique_ptr<PowerModelStateBase> statePtr) = 0;

  /**
   * @brief reportEvent notify the channel of n occurrences of a specific event.
   * The internal count of the channel is cumulative, so each write adds to an
   * internal counter.
   * @param eventId id of the event, as obtained from registerEvent
   * @param n number of occurrences
   */
  virtual void reportEvent(const int eventId, const int n = 1) = 0;

  /**
   * @brief reportState notify the channel of the current state of a module.
   * This method can be called regardless of whether the module state has
   * changed or remains the same.
   * @param stateId id of the module state, as obtained from registerState
   */
  virtual void reportState(const int stateId) = 0;
};

/**
 * @brief class PowerModelChannelInIf input interface. This is used
 * by power modelling modules to obtain the cumulative event-energy,
 * state-current, or event count.
 */
class PowerModelChannelInIf : public virtual sc_core::sc_interface {
 public:
  /**
   * @brief popEventCount pop the event count from an event. This returns the
   * occurrence count and resets the internal counter.
   * @param eventId id of the event, as obtained from registerEvent
   * @retval cumulated count for the specified event
   */
  virtual int popEventCount(const int eventId) = 0;

  /**
   * @brief popEventEnergy pop the event energy from an event. This returns the
   * cumulated energy and resets the internal event counter.
   * @param eventId id of the event, as obtained from registerEvent
   * @param supplyVoltage current supply voltage. Needed by some events to
   * calculate energy consumption.
   * @retval cumulated energy for the specified event
   */
  virtual double popEventEnergy(const int eventId, double supplyVoltage) = 0;

  /**
   * @brief popDynamicEnergy pop the event energy of all events. This returns
   * the cumulated energy and resets all internal event counters.
   * @param supplyVoltage current supply voltage. Needed by some events to
   * calculate energy consumption.
   * @retval Sum of energy consumption for all events, since the last time one
   * of the pop functions was called.
   */
  virtual double popDynamicEnergy(double supplyVoltage) = 0;

  /**
   * @brief getStaticCurrent get the static current in this timestep as a sum of
   * all module-states.
   * @param supplyVoltage supply voltage in the current time step. Used by some
   * module states to determine current consumption.
   * @param clockFrequency clock frequency in the current time step. Used by
   * some module states to determine current consumption.
   * */
  virtual double getStaticCurrent(double supplyVoltage,
                                  double clockFrequency) = 0;

  /**
   * @brief size returns the number of registered events.
   * @retval the number of registered events.
   */
  virtual size_t size() const = 0;
};

// Typedef of ports for convenience
typedef sc_core::sc_port<PowerModelChannelOutIf> PowerModelEventOutPort;
typedef sc_core::sc_port<PowerModelChannelInIf> PowerModelEventInPort;
