/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>

class ClockSourceConsumerIf : public virtual sc_core::sc_interface {
 public:
  /**
   * @brief getPeriod
   * @retval clock period
   */
  virtual const sc_core::sc_time &getPeriod() const = 0;

  /**
   * @brief default_event returns the default event of this channel, used when
   * building the sensitivity list (i.e. "sensitive << channel")
   * @retval default event
   */
  virtual const sc_core::sc_event &default_event() const = 0;

  /**
   * @brief periodChangedEvent returns an event that triggers when the clock
   * period is changed.
   * @retval periodChangedEvent
   */
  virtual const sc_core::sc_event &periodChangedEvent() const = 0;
};

class ClockSourceDriverIf : public ClockSourceConsumerIf {
 public:
  /**
   * @brief setPeriod set clock period. Cancels next clock edge and queues up a
   * new edge according to period
   * @param period new clock period
   */
  virtual void setPeriod(const sc_core::sc_time &period) = 0;

  /**
   * @brief Reset & stop clock source.
   */
  virtual void reset() = 0;
};
