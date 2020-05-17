/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "sd/SpiDevice.hpp"

class DummySpiDevice : public SpiDevice {

public:
  //! Constructor
  DummySpiDevice(const sc_core::sc_module_name nm);

  /**
   * @brief reset Clear control registers.
   *              Also clear shift registers.
   */
  void reset(void) override; 

  /**
   * @brief process Handles tasks upon receving SPI payload.
   */
  void process(void) override;

private:
};
