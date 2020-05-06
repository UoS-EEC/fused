/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <tlm_utils/simple_initiator_socket.h>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "mcu/RegisterFile.hpp"
#include "ps/EventLog.hpp"

/**
 * @brief The eUSCI_B class Modelling operation of eUSCI_Bn
 * serial communication module (SPI/I2C)
 */
class eUSCI_B : public BusTarget {
  SC_HAS_PROCESS(eUSCI_B);

 public:
  tlm_utils::simple_initiator_socket<eUSCI_B> iEusciSocket{"iEusciSocket"};
  sc_core::sc_in<bool> ira{"ira"};  // Interrupt reqest accepted signal
  sc_core::sc_out<bool> irq{"irq"}; // Interrupt request output
  
  // Clock inputs
  sc_core::sc_port<ClockSourceConsumerIf> aclk{"aclk"};
  sc_core::sc_port<ClockSourceConsumerIf> smclk{"smclk"};

  /* ------ Ports ------ */

  /*------ Methods ------*/
  /**
   * @brief eUSCI_B Constructor: initialise registers
   * @param name
   * @param control register start address
   * @param control register end address
   * @param delay Bus access delay
   */
  eUSCI_B(sc_core::sc_module_name name, const uint16_t startAddress,
          const uint16_t endAddress, const sc_core::sc_time delay);

  /**
   * @brief b_transport Blocking reads and writes to the config/control
   * registers.
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /**
   * @brief reset Resets the eUSCI_B control registers to their default
   * power-up values
   */
  virtual void reset(void) override;



 private:
  /* ------ Private variables ------ */
  sc_core::sc_event m_euscibTxEvent{"euscibTxEvent"};
  sc_core::sc_event m_euscibRxEvent{"euscibRxEvent"};
  /* ------ Private methods ------ */

  /**
   * @brief process Performs a serial communication type transaction
   * with an external device; payload properties depends on the content
   * of the config/control registers.
   */
  void process();
  
};
