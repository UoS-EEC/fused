/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <spdlog/spdlog.h>
#include <gdb-server/SimulationControlInterface.hpp>
#include <mcu/Microcontroller.hpp>
#include "utilities/Utilities.hpp"

/**
 * @brief SimulationController Implements an interface to control and interact
 * with simulation from an outside program, e.g. from a debug server.
 * Starts/stops simulation, forwards reads/writes etc.
 */
class SimulationController : public SimulationControlInterface {
 public:
  /* ------ Public methods ------ */
  /**
   * @brief Constructor
   */
  SimulationController(Microcontroller *mcu) : m_mcu(mcu) {}

  virtual ~SimulationController() override {}

  // Control and report
  virtual void kill() override;

  virtual void reset() override { m_mcu->reset(); }
  virtual void stall() override { m_mcu->stall(); }
  virtual void unstall() override { m_mcu->unstall(); }
  virtual bool isStalled() override { return m_mcu->isStalled(); }
  virtual void step() override { m_mcu->step(); }

  // Breakpoints
  virtual void insertBreakpoint(unsigned addr) override {
    m_mcu->insertBreakpoint(addr);
  }
  virtual void removeBreakpoint(unsigned addr) override {
    m_mcu->removeBreakpoint(addr);
  }

  // Register access
  virtual uint32_t readReg(size_t num) override {
    return Utility::ttohl(m_mcu->dbgReadReg(num));
  }
  virtual void writeReg(size_t num, uint32_t value) override {
    m_mcu->dbgWriteReg(num, Utility::htotl(value));
  }

  // Memory access
  virtual bool readMem(uint8_t *out, unsigned addr, size_t len) override {
    return m_mcu->dbgReadMem(out, addr, len);
  }

  virtual bool writeMem(uint8_t *src, unsigned addr, size_t len) override {
    return m_mcu->dbgWriteMem(src, addr, len);
  }

  //  Target info
  virtual uint32_t pcRegNum() override { return m_mcu->pc_regnum(); }
  virtual uint32_t nRegs() override { return m_mcu->n_regs(); }
  virtual uint32_t wordSize() override;

  // Target utilities
  virtual uint32_t htotl(uint32_t hostVal) override {
    return Utility::htotl(hostVal);
  }

  virtual uint32_t ttohl(uint32_t targetVal) override {
    return Utility::ttohl(targetVal);
  }

  // Control debugger
  virtual void stopServer() override { m_stopServer = true; }

  virtual bool shouldStopServer() override { return m_stopServer; }

  virtual bool isServerRunning() override { return m_isServerRunning; }

  virtual void setServerRunning(bool status) override {
    m_isServerRunning = status;
  }

 private:
  /* ------ Private variables ------ */
  Microcontroller *m_mcu;
  bool m_stopServer{false};       //! Simulation stops when this is true
  bool m_isServerRunning{false};  //! Check if debug server is running

  /* ------ Private methods ------ */
};
