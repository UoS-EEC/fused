/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <spdlog/spdlog.h>
#include <stdint.h>
#include <iostream>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockDivider.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Utilities.hpp"

#define SYST_BASE 0xE000E010
#define SYST_END 0xE000E0FF

#define OFS_SYST_CSR 0
#define OFS_SYST_RVR 4
#define OFS_SYST_CVR 8
#define OFS_SYST_CALIB 12

#define SYST_CSR (SYST_BASE + OFS_SYST_CSR)
#define SYST_RVR (SYST_BASE + OFS_SYST_RVR)
#define SYST_CVR (SYST_BASE + OFS_SYST_CVR)
#define SYST_CALIB (SYST_BASE + OFS_SYST_CALIB)

#define SYST_CSR_COUNTFLAG 0x00010000
#define SYST_CSR_TICKINT 0x2
#define SYST_CSR_ENABLE 0x1
#define SYST_CSR_CLKSOURCE 0x4
#define SYST_CALIB_NOREF 0x80000000

#define SYST_EXCEPT_ID 15

class SysTick : public BusTarget {
  SC_HAS_PROCESS(SysTick);

 public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};  //! clock input
  sc_core::sc_out<bool> irq{"irq"};  //! Interrupt request output
  sc_core::sc_in<int> returning_exception{
      "returning_exception"};  //! Signals returning exceptions

  /*------ Methods ------*/

  /**
   * @brief SysTick constructor
   * @param name
   * @param delay Bus access delay
   */
  SysTick(const sc_core::sc_module_name name, const sc_core::sc_time delay);

  /**
   * @brief end_of_elaboration Used to initiate calibration register's TENMS.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief reset Reset registers and member values to their power-on values,
   * and cancel pending expire events.
   */
  virtual void reset(void) override;

  /**
   * @brief b_transport Blocking reads and writes
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;
  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const SysTick &rhs);

 private:
  /* ------ SC events ------ */
  sc_core::sc_event m_expiredEvent{
      "m_expiredEvent"};  //! triggered when the timer has expired
  sc_core::sc_event m_updateIrqEvent{
      "m_updateIrqEvent"};  //! event triggered when irq state should be updated

  /*------ Private variables ------*/
  EventLog::eventId m_triggerEvent;
  sc_core::sc_time m_lastTick{sc_core::SC_ZERO_TIME};
  bool m_setIrq{false};

  /* ------ Private methods ------ */
  /**
   * @brief process Timer operation
   */
  void process();

  /**
   * @brief irqControl Control the interrupt request signal (irq)
   */
  void irqControl();

  /**
   * @brief calcCVR calculate value of Current Value Register
   */
  int calcCVR() const;
};
