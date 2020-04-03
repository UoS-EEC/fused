/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>
#include "mcu/ClockSourceIf.hpp"

class ClockSourceChannel : public ClockSourceDriverIf,
                           public sc_core::sc_module {
 public:
  ClockSourceChannel(sc_core::sc_module_name nm,
                     sc_core::sc_time period = sc_core::SC_ZERO_TIME)
      : sc_core::sc_module(nm), m_period(period) {
    SC_HAS_PROCESS(ClockSourceChannel);
    SC_METHOD(process);
    sensitive << m_nextEdgeEvent;

    if (m_period > sc_core::SC_ZERO_TIME) {
      m_nextEdgeEvent.notify(period);
    }
  }

  virtual const sc_core::sc_event &default_event() const override {
    return m_nextEdgeEvent;
  }

  virtual const sc_core::sc_time &getPeriod() const override {
    return m_period;
  }

  virtual void setPeriod(const sc_core::sc_time &period) {
    if (period == getPeriod()) {
      return;  // Do nothing, period hasn't changed
    }

    m_nextEdgeEvent.cancel();  // Cancel queued edge
    if (period > sc_core::SC_ZERO_TIME) {
      m_nextEdgeEvent.notify(period);  // Queue next edge
    } else {
      // Clock is stopped
    }
    m_periodChangedEvent.notify(sc_core::SC_ZERO_TIME);
    m_period = period;
  }

  virtual const sc_core::sc_event &periodChangedEvent() const override {
    return m_periodChangedEvent;
  }

  virtual void reset() override {
    m_period = sc_core::SC_ZERO_TIME;
    m_nextEdgeEvent.cancel();
    m_periodChangedEvent.cancel();
  }

 private:
  /* ------ Private variables ------ */
  sc_core::sc_time m_period;                             //! Clock period
  sc_core::sc_event m_nextEdgeEvent{"m_nextEdgeEvent"};  //! Edge event
  sc_core::sc_event m_periodChangedEvent{
      "m_periodChangedEvent"};  //! Period changed event

  /* ------ Private functions ------ */

  /**
   * @brief main processing loop, queues up the clock edge event(s)
   * @note Clock starts after period has been set.
   */
  void process() {
    if (m_period > sc_core::SC_ZERO_TIME) {
      m_nextEdgeEvent.notify(m_period);
    }
  }
};
