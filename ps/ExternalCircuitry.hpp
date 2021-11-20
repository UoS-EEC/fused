/*
 * Copyright (c) 2018-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include "ps/Capacitor.hpp"
#include "ps/Converters.hpp"
#include "ps/LinearRegulator.hpp"
#include "ps/PowerSupplies.hpp"
#include "ps/SupplyVoltageSupervisor.hpp"
#include "utilities/Config.hpp"
#include <iostream>
#include <spdlog/spdlog.h>
#include <string>
#include <systemc-ams>
#include <systemc>

/*
 * Description:
 *
 * A module that glues together external ciruitry modeled using SystemC-AMS and
 * provides a discrete event interface to SystemC models.
 *
 * Takes icc, i.e. current consumption, as input.
 *
 * Produces vcc, i.e. supply voltage, as output.
 *
 * Takes keepAlive, a boolean signal that forces the external supply voltage
 * supervisor to stay on, even if the measured voltage is below SVSVoff.
 *
 * Produces v_warn, an sc_logic signal that is asserted when the voltage stored
 * in the capacitor is below the VoltageWarning threshold
 *
 */

SC_MODULE(ExternalCircuitry) {
  // Ports
  sc_core::sc_in<double> icc{"icc"};
  sc_core::sc_out<double> vcc{"vcc"};
  sc_core::sc_in<bool> keepAlive{"keepAlive"};
  sc_core::sc_out_resolved v_warn{"v_warn"};

  // ConstantCurrentSupply supply{"supply"};
  SCA_CTOR(ExternalCircuitry)
      : c("c", Config::get().getDouble("CapacitorValue"),
          Config::get().getDouble("CapacitorInitialVoltage")),
        ldo("ldo",
            /*outputVoltage=*/Config::get().getDouble("PowerSupplyVoltage"),
            /*dropoutVoltage=*/0.02, /*leakageCurrent=*/0.0),
        svs("svs", Config::get().getDouble("SVSVon"),
            Config::get().getDouble("SVSVoff"),
            Config::get().getDouble("VoltageWarning")),
        vOutConverter("vOutConverter",
                      sc_core::sc_time::from_seconds(
                          Config::get().getDouble("PowerModelTimestep"))),
        iOutConverter("iOutConverter",
                      sc_core::sc_time::from_seconds(
                          Config::get().getDouble("PowerModelTimestep"))),
        keepAliveConverter("keepAliveConverter",
                           sc_core::sc_time::from_seconds(
                               Config::get().getDouble("PowerModelTimestep"))),
        vwarnConverter("vwarnConverter",
                       sc_core::sc_time::from_seconds(
                           Config::get().getDouble("PowerModelTimestep"))) {
    // Supply
    supply.i(i_supply);
    supply.v(v_cap_in);

    // Input capacitor
    c_in.v(v_cap_in);
    c_in.i_in(i_supply);
    c_in.i_out(i_boost_in);

    // Boost converter
    boostRegulator.i_in(i_boost_in);
    boostRegulator.v_in(v_cap_in);
    boostRegulator.i_out(i_boost_out);
    boostRegulator.v_out(v_cap);

    // Capacitor
    c.v(v_cap);
    c.i_in(i_boost_out);
    c.i_out(i_out_cap);

    // Supply voltage supervisor
    svs.v_in(v_cap);
    svs.v_out(v_out_svs);
    svs.i_in(i_out_cap);
    svs.i_out(i_out_svs);
    svs.force(svsForce);
    svs.warn(svsWarn);

    // Regulator
    ldo.v_in(v_out_svs);
    ldo.v_out(v_out);
    ldo.i_in(i_out_svs);
    ldo.i_out(i_out);

    // Converters between Discrete Event and Timed Data Flow (i.e. SC/SC-AMS)
    vOutConverter.in(v_out);
    vOutConverter.out(vcc);
    iOutConverter.in(icc);
    iOutConverter.out(i_out);
    keepAliveConverter.in(keepAlive);
    keepAliveConverter.out(svsForce);
    vwarnConverter.in(svsWarn);
    vwarnConverter.out(v_warn);
  }

  // Submodules
  // ConstantPowerSupply supply{"supply"};
  PvCellSupply supply{"supply"};

  Capacitor c;
  Capacitor c_in{"c_in", 10.0e-6};
  BoostRegulator boostRegulator{"boostRegulator"};
  LinearRegulator ldo;
  SupplyVoltageSupervisor svs;
  ScaConverters::TdfToDe<double> vOutConverter;
  ScaConverters::DeToTdf<double> iOutConverter;
  ScaConverters::DeToTdf<bool> keepAliveConverter;
  ScaConverters::TdfToDe<sc_dt::sc_logic> vwarnConverter;

  // Signalt
  sca_tdf::sca_signal<double> i_supply{"i_supply"};
  sca_tdf::sca_signal<double> i_boost_in{"i_boost_in"};
  sca_tdf::sca_signal<double> i_boost_out{"i_boost_out"};

  sca_tdf::sca_signal<double> i_out_cap{"i_out_cap"};
  sca_tdf::sca_signal<double> i_out_svs{"i_out_svs"};

  sca_tdf::sca_signal<double> v_cap{"v_cap"};
  sca_tdf::sca_signal<double> v_cap_in{"v_cap_in"};
  sca_tdf::sca_signal<double> v_out_svs{"v_out_svs"};

  sca_tdf::sca_signal<double> i_out{"i_out_sig"};
  sca_tdf::sca_signal<double> v_out{"v_out_sig"};

  sca_tdf::sca_signal<sc_dt::sc_logic> svsWarn{"svsWarn"};
  sca_tdf::sca_signal<bool> svsForce{"svsForce"};
};
