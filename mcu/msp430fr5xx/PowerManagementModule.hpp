/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string>
#include <systemc>
#include <tlm>
#include <tuple>
#include <vector>
#include "mcu/BusTarget.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/EnergyConsumer.hpp"

class PowerManagementModule : public BusTarget {
 public:
  /* ------ Ports ------ */
  sc_core::sc_out<bool> irq{"irq"};     //! Interrupt output
  sc_core::sc_in<bool> ira{"ira"};      //! Interrupt acknowledge input
  sc_core::sc_in<double> vcc{"vcc"};    //! Supply voltage
  sc_core::sc_out<bool> pwrGood{"on"};  //! True if vcc>vmin, otherwise false
  sc_core::sc_out<double> staticPower{"staticPower"};

  /* ------ Methods ------ */
  /**
   * Default constructor, reads start and end address from msp430xxxx.h
   */
  PowerManagementModule(sc_core::sc_module_name name, sc_core::sc_time delay);

  /**
   * Full constructor
   */
  PowerManagementModule(sc_core::sc_module_name name, sc_core::sc_time delay,
                        unsigned startAddress, unsigned endAddress);

  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

 private:
  /* ------ Internal signals ------ */

  /* ------ Private variables ------ */
  double m_vOn;   //! On voltage threshold
  double m_vOff;  //! Off voltage threshold
  double m_vMax;  //! Maximum operating voltage
  bool m_locked;  //! Indicate if registers are locked
  bool m_isOn;    //! Indicate whether output is on

  std::vector<double> m_bootCurrentTrace;  // Trace of boot current
  double m_bootCurrentTimeResolution;

  /* ------ Private methods ------ */

  /**
   * @brief process Measures vcc. Turns on "on" if vcc > vOn and turns off
   * "on" if vcc < v0ff
   */
  [[noreturn]] void process(void);

  /**
   * @brief reset Reset registers and variables to power-up defaults
   */
  void reset(void) override;
};
