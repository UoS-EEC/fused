/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "boards/Board.hpp"
#include "mcu/MemicMicrocontroller.hpp"
#include "mcu/Microcontroller.hpp"
#include "ps/ExternalCircuitry.hpp"
#include "ps/PowerModelBridge.hpp"
#include "ps/PowerModelChannel.hpp"
#include "sd/Accelerometer.hpp"
#include "sd/Bme280.hpp"
#include "utilities/BoolLogicConverter.hpp"
#include "utilities/Config.hpp"
#include "utilities/IoSimulationStopper.hpp"
#include <systemc-ams>
#include <systemc>

class MemicBoard : public Board {
public:
  // Simple custom reset controller for this board
  SC_MODULE(ResetCtrl) {
  public:
    // Ports
    sc_core::sc_in<double> vcc{"vcc"};
    sc_core::sc_out<bool> nReset{"nReset"};

    SC_CTOR(ResetCtrl) {
      m_vCore = Config::get().getDouble("CpuCoreVoltage");
      SC_METHOD(process);
      sensitive << vcc;
    }

  private:
    double m_vCore;
    void process() { nReset.write(vcc.read() > m_vCore); }
  };

  /* ------ Public methods ------ */
  /**
   * @brief constructor
   */
  MemicBoard(const sc_core::sc_module_name name);

  /**
   * @brief destructor closes vcd files
   */
  ~MemicBoard();

  /**
   * @brief getMicrocontroller get a reference to the microcontroller
   */
  virtual Microcontroller &getMicrocontroller() override;

  /* ------ GPIO pin numbers ------ */
  struct Gpio0PinAssignment {
    static const int IO_SIMULATION_STOPPER = 0;
    static const int WORKLOAD_BEGIN = 1;
    static const int KEEP_ALIVE = 5;
    static const int V_WARN = 31;
  };

  struct Gpio1PinAssignment {
    static const int BME280_CHIP_SELECT = 0;
    static const int ACCELEROMETER_CHIP_SELECT = 1;
    static const int ACCELEROMETER_IRQ = 2;
  };

  /* ------ Channels & signals ------ */
  PowerModelChannel powerModelChannel;
  sc_core::sc_signal<double> vcc{"vcc", 0.0};
  sc_core::sc_signal<double> icc{"icc", 0.0};
  sc_core::sc_signal<bool> nReset{"nReset"};
  sc_core::sc_signal<bool> keepAliveBool{"keepAliveBool"};
  std::array<sc_core::sc_signal_resolved, 32> gpio0Pins;
  std::array<sc_core::sc_signal_resolved, 32> gpio1Pins;

  /* ------ Submodules ------ */
  ResetCtrl resetCtrl{"resetCtrl"};
  MemicMicrocontroller mcu{"mcu"};
  ExternalCircuitry externalCircuitry{"externalCircuitry"};
  Utility::ResolvedInBoolOut keepAliveConverter{"keepAliveConverter"};
  IoSimulationStopper ioSimulationStopper{"ioSimulationStopper"};
  PowerModelBridge powerModelBridge;

  /* ------ External chips ------ */
  Accelerometer accelerometer{"accelerometer"};
  Bme280 bme280{"bme280"};

  /* ------ Tracing ------ */
  sca_util::sca_trace_file *vcdfile;
  sca_util::sca_trace_file *tabfile;
};
