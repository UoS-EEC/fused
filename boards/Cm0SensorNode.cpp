/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "boards/Cm0SensorNode.hpp"
#include "mcu/Cm0Microcontroller.hpp"
#include "mcu/Microcontroller.hpp"
#include "ps/ExternalCircuitry.hpp"
#include "utilities/Config.hpp"
#include "utilities/IoSimulationStopper.hpp"
#include "utilities/SimulationController.hpp"
#include <spdlog/spdlog.h>
#include <systemc-ams>
#include <systemc>
#include <thread>

using namespace sc_core;

Cm0SensorNode::Cm0SensorNode(const sc_module_name name)
    : Board(name),
      powerModelChannel(
          "powerModelChannel", /*logfile=*/
          Config::get().getString("OutputDirectory"),
          sc_time::from_seconds(Config::get().getDouble("LogTimestep"))),
      powerModelBridge("powerModelBridge",
                       sc_time::from_seconds(
                           Config::get().getDouble("PowerModelTimestep"))) {
  /* ------ Bind ------ */
  // Reset
  resetCtrl.vcc.bind(vcc);
  resetCtrl.nReset.bind(nReset);
  mcu.nReset.bind(nReset);

  // GPIO
  for (unsigned i = 0; i < gpioPins.size(); i++) {
    mcu.gpio->pins[i].bind(gpioPins[i]);
  }

  // off-chip serial devices
  bme280.nReset.bind(nReset);
  bme280.chipSelect.bind(gpioPins[GpioPinAssignment::BME280_CHIP_SELECT]);
  bme280.powerModelPort.bind(powerModelChannel);
  mcu.spi->spiSocket.bind(bme280.tSocket);

  accelerometer.nReset.bind(nReset);
  accelerometer.chipSelect.bind(
      gpioPins[GpioPinAssignment::ACCELEROMETER_CHIP_SELECT]);
  accelerometer.irq.bind(gpioPins[GpioPinAssignment::ACCELEROMETER_IRQ]);
  accelerometer.powerModelPort.bind(powerModelChannel);
  mcu.spi->spiSocket.bind(accelerometer.tSocket);

  // Power circuitry
  mcu.powerModelPort.bind(powerModelChannel);
  powerModelBridge.powerModelPort.bind(powerModelChannel);
  powerModelBridge.i_out.bind(icc);
  powerModelBridge.v_in.bind(vcc);
  mcu.vcc.bind(vcc);

  // External circuits (capacitor + supply voltage supervisor etc.)
  externalCircuitry.icc.bind(icc);
  externalCircuitry.vcc.bind(vcc);
  externalCircuitry.v_warn.bind(gpioPins[GpioPinAssignment::V_WARN]);

  // KeepAlive -- bind to IO via converter
  keepAliveConverter.in.bind(gpioPins[GpioPinAssignment::KEEP_ALIVE]);
  keepAliveConverter.out.bind(keepAliveBool);
  externalCircuitry.keepAlive.bind(keepAliveConverter.out);

  // --- Print memory map
  std::cout << "------ MCU construction complete ------\n" << mcu.bus;

  /* ------- Signal tracing ------ */
  // Creates a value-change dump
  vcdfile = sca_util::sca_create_vcd_trace_file(
      (Config::get().getString("OutputDirectory") + "/ext.vcd").c_str());

  for (int i = 0; i < gpioPins.size(); ++i) {
    sca_trace(vcdfile, gpioPins[i], fmt::format("GPIO[{:02d}]", i));
  }

  for (int i = 0; i < mcu.nvic->irq.size(); ++i) {
    sca_trace(vcdfile, mcu.nvic->irq[i], fmt::format("NVIC.irqIn[{:02d}]", i));
  }
  sca_trace(vcdfile, mcu.nvic_pending, "NVIC.pendingIrq");
  sca_trace(vcdfile, mcu.cpu_active_exception, "CPU.ActiveException");
  sca_trace(vcdfile, mcu.cpu_returning_exception, "CPU.ReturningException");
  sca_trace(vcdfile, mcu.spi->irq, "SPI.irq");
  sca_trace(vcdfile, mcu.systick_irq, "SysTick.irq");
  sca_trace(vcdfile, vcc, "vcc");
  sca_trace(vcdfile, icc, "icc");
  sca_trace(vcdfile, nReset, "nReset");
  sca_trace(vcdfile, externalCircuitry.v_cap, "externalCircuitry.v_cap");

  // Creates a csv-like file
  tabfile = sca_util::sca_create_tabular_trace_file(
      (Config::get().getString("OutputDirectory") + "/ext.tab").c_str());

  sca_trace(tabfile, vcc, "vcc");
  sca_trace(tabfile, icc, "icc");
  sca_trace(tabfile, nReset, "nReset");
  sca_trace(tabfile, externalCircuitry.v_cap, "externalCircuitry.v_cap");
  sca_trace(tabfile, externalCircuitry.keepAlive,
            "externalCircuitry.keepAlive");
  sca_trace(tabfile, externalCircuitry.i_supply, "externalCircuitry.i_supply");
}

Cm0SensorNode::~Cm0SensorNode() {
  sca_util::sca_close_vcd_trace_file(vcdfile);
  sca_util::sca_close_tabular_trace_file(tabfile);
}

Microcontroller &Cm0SensorNode::getMicrocontroller() { return mcu; }
