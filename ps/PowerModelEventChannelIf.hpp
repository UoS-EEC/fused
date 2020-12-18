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

/**
 * Power model event channel interfaces
 * ------------------------------------
 *
 *  The event-based powr model is implemented in the channel derived from this
 * base class. Modules register their power modelling events and report event
 * occurrences via a port of type PowerModelEventOutPort. The cumulative event
 * count/energy is then available via the PowerModelEventChannelInIf for power
 * modelling modules.
 *
 */

/**
 * @brief class PowerModelEventChannelOutIf event output interface. This is used
 * by modules to register and report their events for power modelling.
 */
class PowerModelEventChannelOutIf : public virtual sc_core::sc_interface {
 public:
  /**
   * @brief registerEvent register a new power model event. All events must be
   * registered before simulation starts. Trying to register an event during
   * simulation will result in an exception. The returned event id is used for
   * incrementing the count of occurrences of the specified event via write.
   * @param eventPtr unique pointer to an event derived from PowerModelEventBase
   * @retval  assigned event id
   */
  virtual int registerEvent(std::unique_ptr<PowerModelEventBase> eventPtr) = 0;

  /**
   * @brief write notify the channel of n occurrences of a specific event. The
   * internal count of the channel is cumulative, so each write adds to an
   * internal counter.
   * @param eventId id of the event, as obtained from registerEvent
   * @param n number of occurrences since last time write was called.
   */
  virtual void write(const int eventId, const int n = 1) = 0;
};

/**
 * @brief class PowerModelEventChannelInIf event input interface. This is used
 * by modules to obtain the cumulative event-energy or count for power
 * modelling.
 */
class PowerModelEventChannelInIf : public virtual sc_core::sc_interface {
 public:
  /**
   * @brief pop pop the event count from an event. This returns the occurrence
   * count and resets the internal counter.
   * @param eventId id of the event, as obtained from registerEvent
   * @retval cumulated count for the specified event
   */
  virtual int pop(const int eventId) = 0;

  /**
   * @brief pop pop the event energy from an event. This returns the cumulated
   * energy and resets the internal event counter.
   * @param eventId id of the event, as obtained from registerEvent
   * @param supplyVoltage current supply voltage. Needed by some events to
   * calculate energy consumption.
   * @retval cumulated energy for the specified event
   */
  virtual double popEnergy(const int eventId, double supplyVoltage) = 0;

  /**
   * @brief pop pop the event energy of all events. This returns the cumulated
   * energy and resets all internal event counters.
   * @param supplyVoltage current supply voltage. Needed by some events to
   * calculate energy consumption.
   * @retval Sum of energy consumption for all events, since the last time one
   * of the pop functions was called.
   */
  virtual double popEnergy(double supplyVoltage) = 0;

  /**
   * @brief size returns the number of registered events.
   * @retval the number of registered events.
   */
  virtual size_t size() const = 0;
};

// Typedef of ports for convenience
typedef sc_core::sc_port<PowerModelEventChannelOutIf> PowerModelEventOutPort;
typedef sc_core::sc_port<PowerModelEventChannelInIf> PowerModelEventInPort;
