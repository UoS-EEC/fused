/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>

/**
 * @brief The DynamicClock class Extension of sc_clock to provide variable
 *        clock period. New period will be in effect immediately.
 */
class DynamicClock : public sc_core::sc_module {
 public:
  /*------ Ports ------*/
  sc_core::sc_out<bool> out{"out"};

  DynamicClock(sc_core::sc_module_name name, double period,
               sc_core::sc_time_unit tu);

  /**
   * @brief period Get clock period.
   * @return
   */
  sc_core::sc_time period(void);

  void setPeriod(sc_core::sc_time period);

 private:
  /*------ Variables ------*/
  sc_core::sc_time _period;
  sc_core::sc_event nextEdgeEvent{"timeout"};

  /*------ Private methods ------*/
  [[noreturn]] void process(void);
};
