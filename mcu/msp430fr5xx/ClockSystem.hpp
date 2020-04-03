/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/DynamicClock.hpp"

class ClockSystem : public BusTarget {
  SC_HAS_PROCESS(ClockSystem);

 public:
  /*------ Ports ------*/
  sc_core::sc_port<ClockSourceDriverIf> mclk{"mclk"};  //! Master clock
  sc_core::sc_port<ClockSourceDriverIf> smclk{
      "smclk"};  //! Subsystem master clock
  sc_core::sc_port<ClockSourceDriverIf> aclk{"aclk"};      //! Auxillary clock
  sc_core::sc_port<ClockSourceDriverIf> vloclk{"vloclk"};  //! VLO clock
  sc_core::sc_port<ClockSourceDriverIf> modclk{"modclk"};  //! module clock

  /*------ Methods ------*/
  /**
   * @brief ClockSystem Constructor: initialise registers and clocks
   * @param name
   * @param startAddress Peripheral start address
   * @param delay Bus access delay
   */
  ClockSystem(sc_core::sc_module_name name, unsigned startAddress,
              sc_core::sc_time delay);

  /**
   * @brief reset Reset register values to their power-on values on the
   * rising edge of pwrOn
   */
  virtual void reset(void) override;

  /**
   * @brief b_transport Blocking reads and writes
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /*------ Static constants ------*/
 private:
  // Base clocks
  static const sc_core::sc_time VLO_PERIOD;    // 10 kHz
  static const sc_core::sc_time MOD_PERIOD;    // 5 MHz
  static const sc_core::sc_time LFMOD_PERIOD;  // MODCLK / 128
  static const float DCO_FREQ_TABLE[];

  // Reset values
  static const uint16_t CSCTL0_RST = 0x9600;
  static const uint16_t CSCTL1_RST = 0x000c;
  static const uint16_t CSCTL2_RST = 0x0033;
  static const uint16_t CSCTL3_RST = 0x0033;
  static const uint16_t CSCTL4_RST = 0xcdc9;
  static const uint16_t CSCTL5_RST = 0x00c0;
  static const uint16_t CSCTL6_RST = 0x0007;

  // Register masks (write-enabled bits)
  static const uint16_t CSCTL0_MASK = 0xff00;  //! CSCTL0 write mask
  static const uint16_t CSCTL1_MASK = 0x004e;  //! CSCTL1 write mask
  static const uint16_t CSCTL2_MASK = 0x0777;  //! CSCTL2 write mask
  static const uint16_t CSCTL3_MASK = 0x0777;  //! CSCTL3 write mask
  static const uint16_t CSCTL4_MASK = 0xdddb;  //! CSCTL4 write mask
  static const uint16_t CSCTL5_MASK = 0x00c3;  //! CSCTL5 write mask
  static const uint16_t CSCTL6_MASK = 0x00ff;  //! CSCTL6 write mask

 private:
  /*------ Variables ------*/
  bool locked;  //! Whether regs are locked

  /* ------ Private methods ------*/

  void updateClocks(void);

  /**
   * @brief ClockSystem::updateClockPeriods Update clock periods if registers
   *        have changed.
   */
  void updateClockPeriods(void);

  /**
   * @brief ClockSystem::wordWrite Write a word to register file after
   *        applying write masks (to filter reserved bits).
   * @param addr
   * @param value
   */
  void wordWrite(uint16_t addr, uint8_t value);

  /**
   * @brief ClockSystem::byteWrite Write a byte to register file after
   *        applying write mask (to filter reserved bits).
   * @param addr
   * @param value
   */
  void byteWrite(uint16_t addr, uint8_t value);
};
