/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <systemc>
#include "mcu/Microcontroller.hpp"

Microcontroller::Microcontroller(sc_core::sc_module_name nm)
    : sc_core::sc_module(nm) {}
