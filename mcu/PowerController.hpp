/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "mcu/BusTarget.hpp"
#include <array>
#include <systemc>

/**
 * @brief PowerController: memory-mapped peripheral that controls reset signals
 * to other modules. Used for software-based power gating.
 *
 * The output register has a reset value of all 1s, so all modules are powered
 * on by default.
 */
class PowerController : public BusTarget {
  SC_HAS_PROCESS(PowerController);

public:
  /* ------ Ports ------ */
  std::array<sc_core::sc_out<bool>, 32> out;

  /* ------ Public constants ------ */

  // Register addresses
  struct RegisterAddress {
    // clang-format off
    //
    // Output register. Each bit controls its corresponding output. A value of
    // 0 power gates the attached module.
    static const unsigned OUT =                0x00;

    // clang-format on
  };

  /*------ Methods ------*/
  /**
   * @brief Constructor
   * @param name
   * @param startAddress
   */
  PowerController(sc_core::sc_module_name name, unsigned startAddress,
                  unsigned resetValue = 0xffffffff);

  /**
   * @brief reset reset register values
   */
  virtual void reset(void) override;

private:
  /* ------ Private variables ------ */
  sc_core::sc_event m_resetEvent{"m_resetEvent"};

  /* ------ Private methods ------ */
  /**
   * @brief process Set output signals according to register value
   */
  void process();
};
