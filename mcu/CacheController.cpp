/*
 * Copyright (c) 2020-2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache 2.0
 */

#include "include/fused.h"
#include "mcu/BusTarget.hpp"
#include "mcu/CacheController.hpp"
#include "mcu/RegisterFile.hpp"
#include "utilities/Config.hpp"
#include <iostream>
#include <spdlog/spdlog.h>
#include <stdint.h>
#include <string>
#include <systemc>
#include <tlm>
#include <vector>

using namespace sc_core;

CacheController::CacheController(const sc_module_name nm,
                                 const unsigned startAddress)
    : BusTarget(nm, startAddress, startAddress + 0xf) {
  // Build register file
  m_regs.addRegister(OFS_DCACHE_CTRL_CSR, 0);
  m_regs.addRegister(OFS_DCACHE_CTRL_CRNTDIRTY, 0,
                     RegisterFile::AccessMode::READ);
  m_regs.addRegister(
      OFS_DCACHE_CTRL_MAXDIRTY,
      Config::get().contains(std::string(this->name()) + ".MaxModified")
          ? Config::get().getUint(std::string(this->name()) + ".MaxModified")
          : 0xffffffff);
}

void CacheController::end_of_elaboration() {
  BusTarget::end_of_elaboration();
  // Set up methods
  SC_METHOD(process);
  sensitive << nDirtyLines.value_changed_event() << m_writeEvent;

  SC_METHOD(reset);
  sensitive << pwrOn;
}

void CacheController::reset() {
  m_regs.reset();
  m_writeEvent.notify(SC_ZERO_TIME); // Force update of process()
}

void CacheController::process() {
  auto crntDirty = nDirtyLines.read();

  // Flush if necessary
  bool shouldFlush =
      m_regs.testBitMask(OFS_DCACHE_CTRL_CSR, DCACHE_CTRL_FLUSH) &&
      crntDirty > 0;
  shouldFlush |= crntDirty > m_regs.read(OFS_DCACHE_CTRL_MAXDIRTY);
  doFlush.write(shouldFlush);

  // Update registers
  m_regs.write(OFS_DCACHE_CTRL_CRNTDIRTY, crntDirty, /*force=*/true);
  if (!shouldFlush) {
    m_regs.clearBitMask(OFS_DCACHE_CTRL_CSR, DCACHE_CTRL_FLUSH);
  }
}

std::ostream &operator<<(std::ostream &os, const CacheController &rhs) {
  // clang-format off
  os << "<CacheController> " << rhs.name()
     << "\n" << rhs.m_regs;
  // clang-format on
  return os;
}
