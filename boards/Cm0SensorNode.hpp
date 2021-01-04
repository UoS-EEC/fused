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
#include "mcu/Cm0Microcontroller.hpp"
#include "mcu/Microcontroller.hpp"
#include "ps/DynamicEnergyChannel.hpp"
#include "ps/ExternalCircuitry.hpp"
#include "ps/PowerCombine.hpp"
#include "ps/PowerModelChannel.hpp"
#include "sd/Accelerometer.hpp"
#include "sd/Bme280.hpp"
#include "utilities/BoolLogicConverter.hpp"
#include "utilities/Config.hpp"
#include "utilities/IoSimulationStopper.hpp"

class Cm0SensorNode : public Board {
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
  Cm0SensorNode(const sc_core::sc_module_name name);

  /**
   * @brief destructor closes vcd files
   */
  ~Cm0SensorNode();

  /**
   * @brief getMicrocontroller get a reference to the microcontroller
   */
  virtual Microcontroller &getMicrocontroller() override;

  /* ------ GPIO pin numbers ------ */
  struct GpioPinAssignment {
    static const int KEEP_ALIVE = 5;
    static const int V_WARN = 31;
    static const int BME280_CHIP_SELECT = 16;
    static const int ACCELEROMETER_CHIP_SELECT = 17;
    static const int ACCELEROMETER_IRQ = 18;
  };

  /* ------ Channels & signals ------ */
  PowerModelChannel powerModelChannel;
  DynamicEnergyChannel dynamicConsumption{"dynamicConsumption"};
  sc_core::sc_signal<double> staticConsumption{"staticConsumption", 0.0};
  sc_core::sc_signal<double> staticConsumptionBoot{"staticConsumptionBoot",
                                                   0.0};
  sc_core::sc_signal<double> totMcuConsumption{"totMcuConsumption", 0.0};
  sc_core::sc_signal<double> vcc{"vcc", 0.0};
  sc_core::sc_signal<bool> nReset{"nReset"};
  sc_core::sc_signal<bool> keepAliveBool{"keepAliveBool"};
  std::array<sc_core::sc_signal_resolved, 32> gpioPins;

  /* ------ Submodules ------ */
  ResetCtrl resetCtrl{"resetCtrl"};
  Cm0Microcontroller mcu{"mcu"};
  PowerCombine<2, 1> pwrCombinator{"PowerCombine"};
  ExternalCircuitry externalCircuitry{"externalCircuitry"};
  Utility::ResolvedInBoolOut keepAliveConverter{"keepAliveConverter"};

  /* ------ External chips ------ */
  Accelerometer accelerometer{"accelerometer"};
  Bme280 bme280{"bme280"};

  /* ------ Tracing ------ */
  sca_util::sca_trace_file *vcdfile;
  sca_util::sca_trace_file *tabfile;
};
