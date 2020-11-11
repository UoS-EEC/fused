/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <spdlog/spdlog.h>
#include <stdint.h>
#include <array>
#include <iostream>
#include <systemc>
#include <tlm>
#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "ps/EventLog.hpp"
#include "utilities/Utilities.hpp"

#define NVIC_BASE 0xE000E100
#define NVIC_END 0xE000E43C

#define OFS_NVIC_ISER 0
#define OFS_NVIC_ICER 0x80
#define OFS_NVIC_ISPR 0x100
#define OFS_NVIC_ICPR 0x180
#define OFS_NVIC_IPR0 0x300
#define OFS_NVIC_IPR1 0x304
#define OFS_NVIC_IPR2 0x308
#define OFS_NVIC_IPR3 0x30c
#define OFS_NVIC_IPR4 0x310
#define OFS_NVIC_IPR5 0x314
#define OFS_NVIC_IPR6 0x318
#define OFS_NVIC_IPR7 0x31c

#define NVIC_ISER (NVIC_BASE + OFS_NVIC_ISER)
#define NVIC_ICER (NVIC_BASE + OFS_NVIC_ICER)
#define NVIC_ISPR (NVIC_BASE + OFS_NVIC_ISPR)
#define NVIC_ICPR (NVIC_BASE + OFS_NVIC_ICPR)
#define NVIC_IPRn (NVIC_BASE + OFS_NVIC_IPRn)

#define NVIC_EXCEPT_ID_BASE 16

class Nvic : public BusTarget {
  SC_HAS_PROCESS(Nvic);

 public:
  /*------ Ports ------*/
  std::array<sc_core::sc_in<bool>, 32> irq;
  sc_core::sc_out<int> pending{"pending"};
  sc_core::sc_in<int> returning{"returning"};
  sc_core::sc_in<int> active{"active"};

  /*------ Methods ------*/
  /**
   * @brief Nvic constructor
   * @param name
   */
  Nvic(const sc_core::sc_module_name name);

  /**
   * @brief reset Reset registers and member values to their power-on values,
   * and cancel pending expire events.
   */
  virtual void reset(void) override;

  /**
   * @brief Set up processes/threads after module construction complete.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief b_transport Blocking reads and writes
   * @param trans
   * @param delay
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

 private:
  /* ------ SC events ------ */
  sc_core::sc_event m_resetEvent{"resetEvent"};

  /*------ Private variables ------*/
  std::array<bool, 32> m_prevIrq{{false}};
  int m_prevActive{-1};  //! active interrupt in the prev. clk cycle
  unsigned m_swClearPending{0};
  unsigned m_swSetPending{0};

  /* ------ Private methods ------ */
  /**
   * @brief writeOneToClear -- clears oldval bits that are 1 in clearbits
   * @param clearbits bits to clear
   * @param oldval original value
   * @retval  oldval & ~(clearReg)
   */
  uint32_t writeOneToClear(uint32_t clearReg, uint32_t oldval);

  /**
   * @brief writeOneToClear -- sets oldval bits that are 1 in setbits
   * @param setbits bits to set
   * @param oldval original value
   * @retval  oldval | setbits
   */
  uint32_t writeOneToSet(uint32_t clearReg, uint32_t oldval);

  /**
   * @brief process Nvic operation
   */
  void process();

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const Nvic &rhs);
};
