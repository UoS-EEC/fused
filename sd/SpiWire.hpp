/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>

#include "sd/SpiDevice.hpp"

class SpiWire : public SpiDevice {
 public:
  SC_HAS_PROCESS(SpiWire);

  //! Constructor
  SpiWire(const sc_core::sc_module_name nm);

  /**
   * @brief reset Clear control registers. Also clear shift registers.
   */
  virtual void reset(void) override;

  /**
   * @brief process Handles tasks upon receving SPI payload.
   */
  void process(void);
};
