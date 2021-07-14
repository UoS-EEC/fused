/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "libs/make_unique.hpp"
#include "mcu/BankedMemory.hpp"
#include "ps/ConstantCurrentState.hpp"
#include "ps/ConstantEnergyEvent.hpp"
#include "utilities/Config.hpp"
#include <memory>
#include <ostream>
#include <spdlog/fmt/fmt.h>
#include <spdlog/spdlog.h>
#include <systemc>

namespace BankedMemory {
using namespace sc_core;

/* ------ BankedMemory (toplevel) implementation ------*/

BankedMemory::BankedMemory(const sc_core::sc_module_name name,
                           const unsigned memoryStartAddress,
                           const unsigned controlStartAddress, const int nBanks,
                           const int bankSize, bool isVolatile)
    : BusTarget(name, memoryStartAddress,
                memoryStartAddress + nBanks * bankSize - 1),
      m_ctrl("m_ctrl", controlStartAddress, nBanks), m_nBanks(nBanks),
      m_bankSize(bankSize), m_isVolatile(isVolatile) {

  // Construct banks & signals, and hook them up
  for (int i = 0; i < nBanks; ++i) {
    // Bank
    m_banks.push_back(std::make_unique<Bank>(
        fmt::format("bank_{:02d}", i).c_str(), bankSize,
        Config::get().getUint(std::string(this->name()) +
                              ".AutoRetentionDelayCycles"),
        Config::get().getUint(std::string(this->name()) +
                              ".ActiveToRetentionDelayCycles"),
        Config::get().getUint(std::string(this->name()) +
                              ".RetentionToActiveDelayCycles"),
        m_isVolatile));

    // Control signals
    SetOffMode.push_back(std::make_unique<sc_signal<bool>>(
        fmt::format("SetOffMode_{:d}", i).c_str()));
    SetRetentionMode.push_back(std::make_unique<sc_signal<bool>>(
        fmt::format("SetRetentionMode_{:d}", i).c_str()));
    SetActiveMode.push_back(std::make_unique<sc_signal<bool>>(
        fmt::format("SetActiveMode_{:d}", i).c_str()));

    // Status signals
    IsOffMode.push_back(std::make_unique<sc_signal<bool>>(
        fmt::format("IsOffMode_{:d}", i).c_str()));
    IsRetentionMode.push_back(std::make_unique<sc_signal<bool>>(
        fmt::format("IsRetentionMode_{:d}", i).c_str()));
    IsActiveMode.push_back(std::make_unique<sc_signal<bool>>(
        fmt::format("IsActiveMode_{:d}", i).c_str()));

    // Hook-up

    // Bank
    m_banks[i]->powerModelPort.bind(powerModelPort);
    m_banks[i]->clk.bind(systemClk);
    m_banks[i]->nReset.bind(pwrOn);

    m_banks[i]->SetControlMode.bind(SetControlMode);

    m_banks[i]->SetOffMode.bind(*SetOffMode[i]);
    m_banks[i]->SetRetentionMode.bind(*SetRetentionMode[i]);
    m_banks[i]->SetActiveMode.bind(*SetActiveMode[i]);

    m_banks[i]->IsOffMode.bind(*IsOffMode[i]);
    m_banks[i]->IsRetentionMode.bind(*IsRetentionMode[i]);
    m_banks[i]->IsActiveMode.bind(*IsActiveMode[i]);

    // Controller
    m_ctrl.SetOffMode[i]->bind(*SetOffMode[i]);
    m_ctrl.SetRetentionMode[i]->bind(*SetRetentionMode[i]);
    m_ctrl.SetActiveMode[i]->bind(*SetActiveMode[i]);

    m_ctrl.IsOffMode[i]->bind(*IsOffMode[i]);
    m_ctrl.IsRetentionMode[i]->bind(*IsRetentionMode[i]);
    m_ctrl.IsActiveMode[i]->bind(*IsActiveMode[i]);
  }

  // Control mode signal (global)
  m_ctrl.SetControlMode.bind(SetControlMode);
}

void BankedMemory::end_of_elaboration() { BusTarget::end_of_elaboration(); }

void BankedMemory::reset() {}

void BankedMemory::b_transport(tlm::tlm_generic_payload &trans,
                               sc_core::sc_time &delay) {
  // spdlog::info("{:s}::b_transport addr=0x{:08x}, len=0x{:08x}", this->name(),
  // trans.get_address(), trans.get_data_length());

  // Forward transaction for analysis by subscribers
  analysisPort.write(trans);

  // Transfer shouldn't cross bank boundaries
  sc_assert(trans.get_address() / m_bankSize ==
            (trans.get_address() + trans.get_data_length() - 1) / m_bankSize);

  // Should be a read or write
  sc_assert(trans.get_command() == tlm::TLM_WRITE_COMMAND ||
            trans.get_command() == tlm::TLM_READ_COMMAND);

  // Forward transfer to the correct bank
  const int bankIdx = trans.get_address() / m_bankSize;
  m_banks[bankIdx]->transfer(trans.get_address() % m_bankSize,
                             trans.get_data_ptr(), trans.get_data_length(),
                             trans.get_command() == tlm::TLM_WRITE_COMMAND
                                 ? Bank::AccessMode::WRITE
                                 : Bank::AccessMode::READ,
                             delay);
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
}

unsigned BankedMemory::transport_dbg(tlm::tlm_generic_payload &trans) {
  // Should be a read or write
  sc_assert(trans.get_command() == tlm::TLM_WRITE_COMMAND ||
            trans.get_command() == tlm::TLM_READ_COMMAND);

  // Debug transport can cross banks

  int remaining = trans.get_data_length();
  int address = trans.get_address();
  auto *ptr = trans.get_data_ptr();

  while (remaining > 0) {
    const auto bankIdx = address / m_bankSize;
    const auto bankAddr = address % m_bankSize;
    const auto len = std::min(remaining, m_bankSize - bankAddr);

    // Forward transfer to the correct bank
    m_banks[bankIdx]->transfer_dbg(bankAddr, ptr, len,
                                   trans.get_command() == tlm::TLM_WRITE_COMMAND
                                       ? Bank::AccessMode::WRITE
                                       : Bank::AccessMode::READ);
    ptr += len;
    address += len;
    remaining -= len;
  }
  trans.set_response_status(tlm::TLM_OK_RESPONSE);
  return trans.get_data_length();
}

/* ------ Control implementation ------ */
Controller::Controller(const sc_module_name name, const unsigned startAddress,
                       const int nBanks)
    : BusTarget(name, startAddress, startAddress + 8 - 1), m_nBanks(nBanks) {

  // Construct input/output signal vectors
  for (int i = 0; i < m_nBanks; ++i) {
    SetRetentionMode.push_back(std::make_unique<sc_out<bool>>(
        fmt::format("SetRetentionMode_{:02d}", i).c_str()));
    SetActiveMode.push_back(std::make_unique<sc_out<bool>>(
        fmt::format("SetActiveMode_{:02d}", i).c_str()));
    SetOffMode.push_back(std::make_unique<sc_out<bool>>(
        fmt::format("SetOffMode_{:02d}", i).c_str()));

    IsRetentionMode.push_back(std::make_unique<sc_in<bool>>(
        fmt::format("IsRetentionMode_{:02d}", i).c_str()));
    IsActiveMode.push_back(std::make_unique<sc_in<bool>>(
        fmt::format("IsActiveMode_{:02d}", i).c_str()));
    IsOffMode.push_back(std::make_unique<sc_in<bool>>(
        fmt::format("IsOffMode_{:02d}", i).c_str()));
  }

  // Construct register file
  // Auto power mode by default
  m_regs.addRegister(RegisterAddress::CONTROL, 1u);

  // All banks in retention-mode at boot
  m_regs.addRegister(RegisterAddress::POWERMODE, 0x11111111);
}

void Controller::end_of_elaboration() {
  BusTarget::end_of_elaboration();

  //  Register methods
  SC_HAS_PROCESS(Controller);

  SC_METHOD(reset);
  sensitive << pwrOn;

  SC_METHOD(process);
  sensitive << pwrOn << m_writeEvent;
}

void Controller::process() {
  const bool isManual = (m_regs.read(RegisterAddress::CONTROL) &
                         BitMask::CONTROL_CONTROL_MODE_MASK) ==
                        BitMask::CONTROL_CONTROL_MODE_MANUAL;
  SetControlMode.write(isManual);

  if (pwrOn.read()) {
    // Update Set-signals
    for (int i = 0; i < m_nBanks; ++i) {
      const auto powerModeMask = BitMask::POWERMODE_BANK0_MASK << 2 * i;
      SetOffMode[i]->write(powerModeMask == 0);
      SetRetentionMode[i]->write(powerModeMask == 1);
      SetActiveMode[i]->write(powerModeMask == 2);
    }

  } else {
    for (int i = 0; i < m_nBanks; ++i) {
      SetControlMode.write(false);
      SetOffMode[i]->write(true);
      SetRetentionMode[i]->write(false);
      SetActiveMode[i]->write(false);
    }
  }
}

void Controller::reset() {
  if (pwrOn.read() == false) {
    m_regs.reset();
  }
}

void Controller::b_transport(tlm::tlm_generic_payload &trans, sc_time &delay) {

  // Update registers before read
  unsigned powerModeReg = 0;
  for (int i = 0; i < m_nBanks; ++i) {
    if (IsOffMode[i]->read()) {
      powerModeReg |= BitMask::POWERMODE_BANK0_OFF << 2 * i;
    } else if (IsRetentionMode[i]->read()) {
      powerModeReg |= BitMask::POWERMODE_BANK0_RETENTION << 2 * i;
    } else if (IsActiveMode[i]->read()) {
      powerModeReg |= BitMask::POWERMODE_BANK0_ACTIVE << 2 * i;
    }
  }
  m_regs.write(RegisterAddress::POWERMODE, powerModeReg);

  BusTarget::b_transport(trans, delay);
}

/* ------ Bank implementation ------ */
Bank::Bank(const sc_core::sc_module_name name, const int capacity,
           const int autoRetentionDelayCycles,
           const int activeToRetentionDelayCycles,
           const int retentionToActiveDelayCycles, const bool isVolatile)
    : sc_module(name), AutoRetentionDelayCycles(autoRetentionDelayCycles),
      ActiveToRetentionDelayCycles(activeToRetentionDelayCycles),
      RetentionToActiveDelayCycles(retentionToActiveDelayCycles),
      m_isVolatile(isVolatile), m_data(capacity, 0xAA) {}

void Bank::end_of_elaboration() {

  // Register power modelling events & states

  // For the states, use parent name for getting the current consumption. This
  // is done to avoid repeating each energy for each bank in a memory in the
  // config file.

  // Strip off last ".*" from this->name
  std::string parentName =
      std::string(this->name())
          .substr(0, std::string(this->name()).find_last_of("."));

  // Multiply state currents with bank size in  bits
  m_activeStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(
          "active",
          Config::get().getDouble(parentName + ".active") * m_data.size() * 8));
  m_retentionStateId = powerModelPort->registerState(
      this->name(),
      std::make_unique<ConstantCurrentState>(
          "retention", Config::get().getDouble(parentName + ".retention") *
                           m_data.size() * 8));
  m_offStateId = powerModelPort->registerState(
      this->name(), std::make_unique<ConstantCurrentState>(
                        "off", Config::get().getDouble(parentName + ".off") *
                                   m_data.size() * 8));

  m_readEventId = powerModelPort->registerEvent(
      this->name(), std::make_unique<ConstantEnergyEvent>(
                        "read", Config::get().getDouble(parentName + " read")));
  m_writeEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(
          "write", Config::get().getDouble(parentName + " write")));
  m_retentionToActiveEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(
          "retentionToActive",
          Config::get().getDouble(parentName + " retentionToActive")));
  m_activeToRetentionEventId = powerModelPort->registerEvent(
      this->name(),
      std::make_unique<ConstantEnergyEvent>(
          "activeToRetention",
          Config::get().getDouble(parentName + " activeToRetention")));

  // Register SC_METHODs
  SC_HAS_PROCESS(Bank);
  SC_THREAD(process);

  SC_METHOD(reset);
  sensitive << nReset.neg();
}

void Bank::process() {
  // Initial state
  if (SetControlMode.read() == true /* Manual mode */) {
    if (SetActiveMode.read() == true) {
      powerModelPort->reportEvent(m_retentionToActiveEventId);
      m_powerMode = PowerMode::ACTIVE;
    } else if (SetRetentionMode.read() == true) {
      m_powerMode = PowerMode::RETENTION;
    } else {
      m_powerMode == PowerMode::OFF;
    }
  } else { /* Auto mode*/
    m_powerMode = PowerMode::RETENTION;
  }

  while (true) {
    if (nReset.read()) {
      if (SetControlMode.read() == true /* Manual */) {
        // Exactly one mode should be selected
        sc_assert(SetActiveMode.read() + SetRetentionMode.read() +
                      SetOffMode.read() ==
                  1);

        // Update power mode and outputs after delays
        if (SetOffMode.read() && m_powerMode != PowerMode::OFF) {
          wait(ActiveToRetentionDelayCycles * clk->getPeriod());
          m_powerMode = PowerMode::OFF;
        } else if (SetRetentionMode.read() &&
                   m_powerMode != PowerMode::RETENTION) {
          powerModelPort->reportEvent(m_activeToRetentionEventId);
          wait(ActiveToRetentionDelayCycles * clk->getPeriod());
          m_powerMode = PowerMode::RETENTION;
        } else if (SetActiveMode.read() && m_powerMode != PowerMode::ACTIVE) {
          powerModelPort->reportEvent(m_retentionToActiveEventId);
          // Enter active state immediately to start consuming "active power"
          // while powering up.
          powerModelPort->reportState(m_activeStateId);
          wait(RetentionToActiveDelayCycles * clk->getPeriod());
          m_powerMode = PowerMode::ACTIVE;
        }

      } else if (SetControlMode.read() == false /* Auto */) {
        // Maximum one mode should be selected
        sc_assert(m_setActive + m_setRetention <= 1);

        // Update power mode after delay, if changed
        if (m_setActive) { // && m_powerMode != PowerMode::ACTIVE) {
          powerModelPort->reportEvent(m_retentionToActiveEventId);
          // Enter active state immediately to start consuming "active power"
          // while powering up.
          powerModelPort->reportState(m_activeStateId);
          wait(RetentionToActiveDelayCycles * clk->getPeriod());
          m_powerMode = PowerMode::ACTIVE;
          m_setActive = false;

          // Start auto-retention timer (in case there was only 1 access)
          m_modeChangedEvent.notify(AutoRetentionDelayCycles *
                                    clk->getPeriod());
          m_setRetention = true;
        } else if (m_setRetention) { // && m_powerMode != PowerMode::RETENTION)
                                     // {
          powerModelPort->reportEvent(m_activeToRetentionEventId);
          wait(ActiveToRetentionDelayCycles * clk->getPeriod());
          m_powerMode = PowerMode::RETENTION;
          m_setRetention = false;
        }
      }

      // Set status signals & report state
      switch (m_powerMode) {
      case PowerMode::ACTIVE:
        IsOffMode.write(false);
        IsRetentionMode.write(false);
        IsActiveMode.write(true);
        powerModelPort->reportState(m_activeStateId);
        break;
      case PowerMode::RETENTION:
        IsOffMode.write(false);
        IsRetentionMode.write(true);
        IsActiveMode.write(false);
        powerModelPort->reportState(m_retentionStateId);
        break;
      case PowerMode::OFF:
        IsOffMode.write(true);
        IsRetentionMode.write(false);
        IsActiveMode.write(false);
        powerModelPort->reportState(m_offStateId);
        break;
      }

      // Wait for next change
      if (SetControlMode.read() == true /*Manual*/) {
        wait(SetControlMode | SetActiveMode | SetRetentionMode | SetOffMode |
             nReset);
      } else {
        wait(SetControlMode.value_changed_event() | m_modeChangedEvent |
             nReset.negedge_event());
      }
    } else if (!nReset.read()) {
      // Powered off
      IsOffMode.write(true);
      IsRetentionMode.write(false);
      IsActiveMode.write(false);
      powerModelPort->reportState(m_offStateId);
      wait(nReset.posedge_event());

      // Power-up/initial state
      if (SetControlMode.read() == true /* Manual mode */) {
        if (SetActiveMode.read() == true) {
          m_powerMode = PowerMode::ACTIVE;
          powerModelPort->reportEvent(m_retentionToActiveEventId);
        } else if (SetRetentionMode.read() == true) {
          m_powerMode = PowerMode::RETENTION;
        } else {
          m_powerMode == PowerMode::OFF;
        }
      } else { /* Auto mode*/
        m_powerMode = PowerMode::RETENTION;
      }
    }
  }
} // namespace BankedMemory

void Bank::reset() {
  m_setActive = false;
  m_setRetention = false;

  if (m_isVolatile) {
    std::fill(m_data.begin(), m_data.end(), 0xAA);
  }
}

void Bank::transfer(unsigned address, uint8_t *buf, const int len,
                    const AccessMode accessMode, sc_time &delay) {
  // Check for valid transfer
  sc_assert(address + len <= m_data.size());

  // Reset timers & flags
  m_modeChangedEvent.cancel();

  // Set delay and timer-events
  switch (m_powerMode) {
  case PowerMode::ACTIVE:
    delay += clk->getPeriod();
    if (SetControlMode.read() == false /*Auto*/) {
      m_setRetention = true;
      m_modeChangedEvent.notify(AutoRetentionDelayCycles * clk->getPeriod());
    }
    break;
  case PowerMode::RETENTION:
    if (SetControlMode.read()) { /* Manual */
      SC_REPORT_FATAL(this->name(), "Bank::transfer access while power mode is "
                                    "RETENTION and control mode is MANUAL");

    } else { /* Auto */
      delay += (RetentionToActiveDelayCycles + 1) * clk->getPeriod();
      m_setActive = true;
      m_modeChangedEvent.notify(SC_ZERO_TIME);
    }
    break;
  case PowerMode::OFF:
    SC_REPORT_FATAL(this->name(),
                    "Bank::transfer access while power mode is OFF");
    break;
  }

  // Copy data
  if (accessMode == AccessMode::READ) {
    std::memcpy(buf, &m_data[address], len);
  } else if (accessMode == AccessMode::WRITE) {
    std::memcpy(&m_data[address], buf, len);
  }

  // Report number of words accessed
  if (accessMode == AccessMode::READ) {
    powerModelPort->reportEvent(m_readEventId, std::max(1, len / 4));
  } else if (accessMode == AccessMode::WRITE) {
    powerModelPort->reportEvent(m_writeEventId, std::max(1, len / 4));
  }
}

void Bank::transfer_dbg(unsigned address, uint8_t *buf, const int len,
                        const AccessMode accessMode) {
  // Check for valid transfer
  sc_assert(address + len <= m_data.size());

  // Copy data
  if (accessMode == AccessMode::READ) {
    std::memcpy(buf, &m_data[address], len);
  } else if (accessMode == AccessMode::WRITE) {
    std::memcpy(&m_data[address], buf, len);
  }
}

std::ostream &operator<<(std::ostream &os, const Bank &rhs) {
  os << "<BankedMemory::Bank> " << rhs.name() << "\n";
  os << "\tsize(bytes) = " << rhs.m_data.size() << "\n";
  os << "\tcontrol mode = " << (rhs.SetControlMode.read() ? "MANUAL" : "AUTO")
     << "\n";
  os << "\tpower mode = "
     << (rhs.m_powerMode == Bank::PowerMode::OFF
             ? "OFF"
             : rhs.m_powerMode == Bank::PowerMode::RETENTION ? "RETENTION"
                                                             : "ACTIVE");
  return os;
}

} // namespace BankedMemory
