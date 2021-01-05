/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <iostream>
#include <systemc>
#include <tlm>
#include "mcu/ClockSourceIf.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/EventLog.hpp"
#include "ps/PowerModelChannelIf.hpp"

class BusTarget : public sc_core::sc_module, public tlm::tlm_fw_transport_if<> {
 public:
  /* ------ Ports ------ */
  //! Bus clock
  sc_core::sc_port<ClockSourceConsumerIf> systemClk{"systemClk"};

  //! TLM bus socket
  tlm::tlm_target_socket<> tSocket;

  //! Indicates if power to this target is on
  sc_core::sc_in<bool> pwrOn{"pwrOn"};

  //! Event-port for logging and reporting dynamic power consumption
  PowerModelEventOutPort powerModelPort{"powerModelPort"};

  /* ------ Public methods ------ */
  BusTarget(const sc_core::sc_module_name name, const unsigned startAddress,
            const unsigned endAddress);

  /**
   * @brief SystemC callback, used here to register power modelling events.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief b_transport transaction.
   * Default implementation writes/reads to/from m_regs
   * @param trans
   * @return
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay);

  /**
   * @brief transport_dbg Transaction without affecting simulation time.
   * Default implementation writes/reads to/from m_regs
   * @param trans
   * @return
   */
  virtual unsigned int transport_dbg(tlm::tlm_generic_payload &trans);

  /**
   * @brief reset Resets to power-up defaults.
   */
  virtual void reset(void) = 0;

  /**
   * @brief inRange Check whether an address is in range for this target.
   *        The bus is assumed to decrement the address before sending the
   *        transaction to bus targets, so we check that the address fits
   *        within this peripherals range (0 to (startAddress - endAddress).
   * @param addr Address to check
   * @return True if address is within this target's address range, false
   *         otherwise
   */
  bool inRange(const unsigned int addr) const {
    return (addr <= (m_startAddress - m_endAddress));
  }

  /**
   * @brief startAddress getter
   */
  unsigned startAddress() const { return m_startAddress; }

  /**
   * @brief endAddress getter
   */
  unsigned endAddress() const { return m_endAddress; }

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const BusTarget &rhs);

  /*------ Dummy methods --------------------------------------------------*/

  // dummy method
  [[noreturn]] virtual tlm::tlm_sync_enum nb_transport_fw(
      tlm::tlm_generic_payload &trans[[maybe_unused]],
      tlm::tlm_phase &phase[[maybe_unused]],
      sc_core::sc_time &delay[[maybe_unused]]) {
    SC_REPORT_ERROR(this->name(), "not implemented");
    exit(1);
  }

  // dummy method
  [[noreturn]] virtual bool get_direct_mem_ptr(
      tlm::tlm_generic_payload &trans[[maybe_unused]],
      tlm::tlm_dmi &data[[maybe_unused]]) {
    SC_REPORT_ERROR(this->name(), "not implemented");
    exit(1);
  }

 protected:
  const unsigned int m_startAddress;
  const unsigned int m_endAddress;
  RegisterFile m_regs;
  int m_readEventId{-1};
  int m_writeEventId{-1};
  EventLog &m_elog;

  //! Events triggered on bus access via b_transport -- not transport_dbg!
  sc_core::sc_event m_readEvent{"readEvent"};    //! Triggered on read access
  sc_core::sc_event m_writeEvent{"writeEvent"};  //! Triggered on write access

  // sc_core::sc_signal<float> m_staticCurrent{"m_staticCurrent", 0.0f};
};
