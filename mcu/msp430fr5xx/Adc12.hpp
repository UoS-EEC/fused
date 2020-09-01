/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockDivider.hpp"
#include "mcu/ClockMux.hpp"
#include "mcu/ClockSourceChannel.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/EventLog.hpp"

/**
 * @brief The Adc12 class Modelling specific mode of ADC12 module
 *
 */
class Adc12 : public BusTarget {
  SC_HAS_PROCESS(Adc12);

 public:
  /* ------ Ports ------ */
  // Analog
  sc_core::sc_in<double> vcc{"vcc"};    //! Supply voltage
  sc_core::sc_in<double> vref{"vref"};  //! Reference voltage

  // Clock inputs
  sc_core::sc_port<ClockSourceConsumerIf> modclk{"modclk"};
  sc_core::sc_port<ClockSourceConsumerIf> aclk{"aclk"};
  sc_core::sc_port<ClockSourceConsumerIf> mclk{"mclk"};
  sc_core::sc_port<ClockSourceConsumerIf> smclk{"smclk"};

  // Interrupt
  sc_core::sc_out<bool> irq{"irq"};  //! Interrupt output

  /* ------ Submodules ------ */
  ClockMux<4> clkMux{"clkMux"};     //! Input clock mux
  ClockDivider clkDiv{"smclkDiv"};  //! Clock divider

  /*------ Internal signals/channels ------*/
  sc_signal<int> clkDivAmount{"aclkDivAmount",
                              1};  //! Amount to divide source clock by
  sc_signal<int> clkMuxSelect{"clkMuxSelect", 0};  //! Mux source select
  ClockSourceChannel muxOut{"muxOut"};             //! Mux output clock
  ClockSourceChannel samplingClock{
      "samplingClock"};  //! Sampling Clock (ADC12CLK)

  /*------ Static constants ------*/
  // Mux control
  static const int MODCLK_SEL;  //! Select MODCLK as source
  static const int ACLK_SEL;    //! Select ACLK as source
  static const int MCLK_SEL;    //! Select MCLK as source
  static const int SMCLK_SEL;   //! Select SCMCLK as source

  /*------ Methods ------*/
  /**
   * @brief Adc12 Constructor: initialise registers
   * @param name
   * @param delay Bus access delay
   */
  Adc12(const sc_core::sc_module_name name);

  /**
   * @brief b_transport Blocking reads and writes
   * @param trans
   * @param delay
   */
  void b_transport(tlm::tlm_generic_payload &trans,
                   sc_core::sc_time &delay) override;

  /**
   * @brief reset Resets the ADC registers to their default power-up values
   * on the rising edge of pwrOn
   * and shuts off the ADC.
   */
  virtual void reset(void) override;

  /**
   * @brief end_of_elaboration used to register SC_METHODs and build sensitivity
   * list.
   */
  void end_of_elaboration();

 private:
  /* ------ Private variables ------ */
  EventLog::eventId m_sampleEvent;
  bool m_active{false};

  /* ------ SC events ------ */
  sc_core::sc_event samplingClockUpdateEvent{"samplingClockUpdateEvent"};
  sc_core::sc_event modeEvent{"modeEvent"};

  /* ------ Private methods ------ */
  /**
   * @brief process main process loop: sample, convert, request interrupts etc.
   */
  void process();

  /**
   * @brief updateClkSource Select clock source and divider. In the current
   * implementation, the source clock is divided by the ADC predivider, divider,
   * sampling time and conversion time, i.e. we simplify the sample-and-hold
   * procedure to a single instantaneous sample.
   */
  void updateClkSource();
};
