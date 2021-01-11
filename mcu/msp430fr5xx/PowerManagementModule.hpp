/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/fmt/fmt.h>
#include <memory>
#include <string>
#include <systemc>
#include <tlm>
#include <tuple>
#include <vector>
#include "mcu/BusTarget.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/PowerModelStateBase.hpp"

class PowerManagementModule : public BusTarget {
 public:
  /* ------ Ports ------ */
  sc_core::sc_out<bool> irq{"irq"};     //! Interrupt output
  sc_core::sc_in<bool> ira{"ira"};      //! Interrupt acknowledge input
  sc_core::sc_in<double> vcc{"vcc"};    //! Supply voltage
  sc_core::sc_out<bool> pwrGood{"on"};  //! True if vcc>vmin, otherwise false

  /* ------ Methods ------ */
  /**
   * Default constructor, reads start and end address from msp430xxxx.h
   */
  PowerManagementModule(sc_core::sc_module_name name);

  /**
   * Full constructor
   */
  PowerManagementModule(sc_core::sc_module_name name, unsigned startAddress,
                        unsigned endAddress);

  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /**
   * @brief set up methods, sensitivity, and register power model events and
   * states
   */
  virtual void end_of_elaboration() override;

 private:
  /* ------ Internal classes ------ */

  /* Specialized power state for reporting boot current. */
  class BootCurrentState : public PowerModelStateBase {
   public:
    BootCurrentState(const std::string name) : PowerModelStateBase(name) {}

    void setCurrent(double current) { m_current = current; }

    virtual double calculateCurrent([
        [maybe_unused]] double supplyVoltage) const override {
      return m_current;
    }

    virtual std::string toString() const override {
      return fmt::format(
          FMT_STRING("{:s} <BootCurrentState> current={:.6} nA (variable)"),
          name, m_current * 1e9);
    }

   private:
    double m_current{0.0};
  };

  std::shared_ptr<BootCurrentState> m_bootCurrentState;

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
