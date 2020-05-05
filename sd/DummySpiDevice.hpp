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
  // Control registers
  struct c_reg {        
    const size_t addr;
    uint8_t val;
  };

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

  /**
   * @brief n_regs Get the number of memory mapped registers.
   */
  uint32_t n_regs(void) override;

private:
  std::vector<c_reg> c_regs;
};
