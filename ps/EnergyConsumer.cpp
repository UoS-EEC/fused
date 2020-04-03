/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "ps/EnergyConsumer.hpp"

using namespace sc_core;

EnergyConsumer::EnergyConsumer(sc_module_name name) : sc_module(name) {
  staticConsumption.bind(m_staticConsumption);
}

void EnergyConsumer::consumeDynamic(float energy) {
  dynamicConsumption.write(energy);
}

void EnergyConsumer::setStaticConsumption(float current) {
  m_staticConsumption.write(current);
}
