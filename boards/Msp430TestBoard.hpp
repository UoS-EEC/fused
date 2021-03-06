/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <systemc-ams>
#include <systemc>
#include "boards/Board.hpp"
#include "mcu/Microcontroller.hpp"
#include "mcu/Msp430Microcontroller.hpp"
#include "ps/ExternalCircuitry.hpp"
#include "ps/PowerModelBridge.hpp"
#include "ps/PowerModelChannel.hpp"
#include "sd/SpiLoopBack.hpp"
#include "utilities/BoolLogicConverter.hpp"
#include "utilities/Config.hpp"
#include "utilities/IoSimulationStopper.hpp"

class Msp430TestBoard : public Board {
 public:
  /* ------ Public methods ------ */
  /**
   * @brief constructor
   */
  Msp430TestBoard(const sc_core::sc_module_name name);

  /**
   * @brief destructor closes vcd files
   */
  ~Msp430TestBoard();

  /**
   * @brief getMicrocontroller get a reference to the microcontroller
   */
  virtual Microcontroller &getMicrocontroller() override;

  /* ------ Channels & signals ------ */
  PowerModelChannel powerModelChannel;
  sc_core::sc_signal<double> vcc{"vcc", 0.0};
  sc_core::sc_signal<double> icc{"icc", 0.0};
  sc_core::sc_signal<bool> nReset{"nReset"};
  sc_core::sc_signal_resolved chipSelectSpiWire{"chipSelectSpiWire",
                                                sc_dt::SC_LOGIC_0};
  sc_core::sc_signal<bool> keepAliveBool{"keepAliveBool"};

  // IO pins
  std::array<sc_signal_resolved, 16> DIOAPins;
  std::array<sc_signal_resolved, 16> DIOBPins;
  std::array<sc_signal_resolved, 16> DIOCPins;
  std::array<sc_signal_resolved, 16> DIODPins;

  /* ------ Submodules ------ */
  Msp430Microcontroller mcu{"mcu"};
  SpiLoopBack spiLoopBack{"spiLoopBack"};
  ExternalCircuitry externalCircuitry{"externalCircuitry"};
  Utility::ResolvedInBoolOut keepAliveConverter{"keepAliveConverter"};
  IoSimulationStopper simStopper{"PA2Stopper"};
  PowerModelBridge powerModelBridge{"powerModelBridge"};

  /* ------ Tracing ------ */
  sca_util::sca_trace_file *vcdfile;
  sca_util::sca_trace_file *tabfile;
};
