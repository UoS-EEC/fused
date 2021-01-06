/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockDivider.hpp"
#include "mcu/ClockMux.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/ClockSourceIf.hpp"

class TimerA : public BusTarget {
  SC_HAS_PROCESS(TimerA);

 public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceConsumerIf> aclk{"aclk"};  //! Auxillary clock in
  sc_core::sc_port<ClockSourceConsumerIf> smclk{
      "smclk"};                      //! Subsystem Master Clock
  sc_core::sc_in<bool> ira{"ira"};   //! Interrupt request accepted
  sc_core::sc_out<bool> irq{"irq"};  //! Interrupt request output
  sc_core::sc_out<bool> dmaTrigger{"dmaTrigger"};  //! DMA trigger out

  /*------ Methods ------*/

  /**
   * @brief TimerA constructor
   * @param name
   * @param startAddress Peripheral start address
   */
  TimerA(sc_core::sc_module_name name, unsigned startAddress);

  /**
   * @brief reset Reset registers and member values to their power-on values.
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
   * @brief set up methods, sensitivity, and register power model events and
   * states
   */
  virtual void end_of_elaboration() override;

 public:
  /*------ Internal signals/channels ------*/
  sc_signal<int> clkDivAmount{"aclkDivAmount",
                              1};  //! Amount to divide source clock by
  sc_signal<int> clkMuxSelect{"clkMuxSelect", 0};  //! Mux source select
  ClockSourceChannel muxOut{"muxOut"};             //! Mux output clock
  ClockSourceChannel timerClock{
      "timerClock"};  //! Timer Clock (final clock used by timer)

  /*------ Submodules ------*/
  ClockMux<2> clkMux{"clkMux"};     //! Input clock mux
  ClockDivider clkDiv{"smclkDiv"};  //! Clock divider

  /*------ Static constants ------*/
  // Mux control
  static const int ACLK_SEL;   //! Select ACLK as source
  static const int SMCLK_SEL;  //! Select SMCLK as source

 private:
  /*------ Private variables ------*/
  bool direction;  //! Counting direction
  sc_core::sc_event
      sourceChangeEvent;  //! Triggered when clock source is changed.

  int m_triggerEventId{-1};

  /* ------ Private methods ------ */
  /**
   * @brief process Timer operation
   */
  void process();

  /**
   * @brief updateClkSource Update source clock
   */
  void updateClkSource();
};
