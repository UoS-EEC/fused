/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <spdlog/spdlog.h>
#include <gdb-server/SimulationControlInterface.hpp>
#include <mcu/Microcontroller.hpp>
#include "utilities/SimulationController.hpp"
#include "utilities/Utilities.hpp"

void SimulationController::kill() {
  // Unstall systemc and kill simulation
  m_mcu->unstall();
  m_mcu->kill();
}

uint32_t SimulationController::wordSize() { return TARGET_WORD_SIZE; }
