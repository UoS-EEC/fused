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
#include <vector>

#include "sd/SpiDevice.hpp"

/**
 * Base class for SPI wire.
 */
class SpiLoopBack : public SpiDevice {
  SC_HAS_PROCESS(SpiLoopBack);

 public:
  /* ------ Public methods ------ */

  //! Constructor
  explicit SpiLoopBack(sc_core::sc_module_name nm);

  /**
   * @brief b_transport TLM blocking transaction method.
   * @param trans tlm_generic_payload for SPI packet
   * @param delay
   */
  void b_transport(tlm::tlm_generic_payload &trans, sc_core::sc_time &delay);

 protected:
  /* ------ Protected methods ------ */

  /**
   * @brief reset Reset the device registers, including
   *              the SPI shift registers.
   */
  void reset() override;
};
