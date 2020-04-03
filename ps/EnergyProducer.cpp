/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ps/EnergyProducer.hpp"

// EnergyProducer::EnergyProducer(sc_module_name name) : sc_module(name) {}

void EnergyProducer::produceDynamic(double energy) {
  dynamicProduction->write(energy);
}

void EnergyProducer::setStaticProduction(double current) {
  staticProduction.write(current);
}
