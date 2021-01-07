/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <systemc-ams>
#include <systemc>
#include <thread>
#include "boards/Cm0TestBoard.hpp"
#include "mcu/Cm0Microcontroller.hpp"
#include "mcu/Microcontroller.hpp"
#include "ps/ExternalCircuitry.hpp"
#include "utilities/Config.hpp"
#include "utilities/IoSimulationStopper.hpp"
#include "utilities/SimulationController.hpp"

using namespace sc_core;

Cm0TestBoard::Cm0TestBoard(const sc_module_name name)
    : Board(name),
      powerModelChannel(
          "powerModelChannel", /*logfile=*/
          Config::get().getString("OutputDirectory"),
          sc_time::from_seconds(Config::get().getDouble("LogTimeStep"))) {
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
  spiLoopBack.nReset.bind(nReset);
  spiLoopBack.chipSelect.bind(chipSelectDummySpi);
  spiLoopBack.powerModelPort.bind(powerModelChannel);
  mcu.spi->spiSocket.bind(spiLoopBack.tSocket);

  // Power circuitry
  mcu.powerModelPort.bind(powerModelChannel);
  powerModelBridge.powerModelPort.bind(powerModelChannel);
  powerModelBridge.i_out.bind(icc);
  powerModelBridge.v_in.bind(vcc);
  mcu.vcc.bind(vcc);

  // External circuits (capacitor + supply voltage supervisor etc.)
  externalCircuitry.i_out.bind(icc);
  externalCircuitry.vcc.bind(vcc);
  externalCircuitry.v_warn.bind(gpioPins[31]);

  // KeepAlive -- bind to IO via converter
  keepAliveConverter.in.bind(gpioPins[5]);
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

  // Creates a csv-like file
  tabfile = sca_util::sca_create_tabular_trace_file(
      (Config::get().getString("OutputDirectory") + "/ext.tab").c_str());

  sca_trace(tabfile, powerModelBridge.v_in, "vcc");
  sca_trace(tabfile, powerModelBridge.i_out, "icc");
  sca_trace(tabfile, nReset, "nReset");
  sca_trace(tabfile, externalCircuitry.v_cap, "externalCircuitry.v_cap");
  sca_trace(tabfile, externalCircuitry.keepAlive,
            "externalCircuitry.keepAlive");
  sca_trace(tabfile, externalCircuitry.i_supply, "externalCircuitry.i_supply");
}

Cm0TestBoard::~Cm0TestBoard() {
  sca_util::sca_close_vcd_trace_file(vcdfile);
  sca_util::sca_close_tabular_trace_file(tabfile);
}

Microcontroller &Cm0TestBoard::getMicrocontroller() { return mcu; }
