/*
 * Copyright (c) 2018-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#pragma once

#include <systemc-ams>
#include <systemc>

/**
 * Utility SC-AMS modules for port conversion etc.
 *
 */
namespace ScaConverters {

/**
 * TdfToDe:
 * Convert an SC-AMS TDF input to a normal systemc (discrete event) output.
 * Performs the conversion at a configurable timestep.
 */
template <typename T> SCA_TDF_MODULE(TdfToDe) {
  sca_tdf::sca_in<T> in{"in"};
  sca_tdf::sc_out<T> out{"out"};

  TdfToDe(const sc_core::sc_module_name name, sc_core::sc_time timestep)
      : sca_tdf::sca_module(name), m_timestep(timestep) {}

  void set_attributes() { set_timestep(m_timestep); }

  void processing() { out.write(in.read()); }

private:
  const sc_core::sc_time m_timestep;
};

/**
 * DeToTdf:
 * Convert a normal systemc (discrete event) input to an SC-AMS TDF output.
 * Performs the conversion at a configurable timestep.
 */
template <typename T> SCA_TDF_MODULE(DeToTdf) {
  sca_tdf::sc_in<T> in{"in"};
  sca_tdf::sca_out<T> out{"out"};

  DeToTdf(const sc_core::sc_module_name name, sc_core::sc_time timestep)
      : sca_tdf::sca_module(name), m_timestep(timestep) {}

  void set_attributes() { set_timestep(m_timestep); }

  void processing() { out.write(in.read()); }

private:
  const sc_core::sc_time m_timestep;
};

} // namespace ScaConverters
