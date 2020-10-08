/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>
#include <ostream>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/EventLog.hpp"

/**
 * @brief The Gpio class : simple unidirectional output-only "IO" port.
 */
class Gpio : public BusTarget {
  SC_HAS_PROCESS(Gpio);

 public:
  /* ------ Ports ------ */
  /* ------ Ports ------ */
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};
  sc_core::sc_out<bool> irq{"irq"};  //! Interrupt request output
  sc_core::sc_in<int> active_exception{
      "active_exception"};  //! Signals exception taken by cpu
  std::array<sc_core::sc_inout_resolved, 32> pins;

  /*------ Methods ------*/
  /**
   * @brief DigitalIo Constructor: initialise registers
   * @param name
   */
  Gpio(const sc_core::sc_module_name name);

  /**
   * @brief Set up systemc methods.
   */
  virtual void before_end_of_elaboration() override;

  /**
   * @brief reset Resets the IO registers to their default power-up values
   * on the rising edge of pwrOn.
   */
  virtual void reset(void) override;

  /**
   * @brief ostream operator<< for debug printout.
   */
  friend std::ostream& operator<<(std::ostream& os, const Gpio& rhs);

 private:
  /* ------ Private variables ------ */
  bool m_setIrq{false};     //! 1 if irq should be set
  unsigned m_lastState{0};  //! Last pin state, used to check for edges
  sc_core::sc_event m_updateIrqEvent{"updateIrqEvent"};
  EventLog::eventId m_pinPosEdge;
  EventLog::eventId m_pinNegEdge;

  /* ------ Private methods ------ */
  /**
   * @brief process Set all output signals according to register value
   */
  void process();

  /**
   * @brief irqControl control irq signalling
   */
  void irqControl();
};
