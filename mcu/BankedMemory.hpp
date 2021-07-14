/*
 * Copyright (c) 2021, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "mcu/BusTarget.hpp"
#include "mcu/ClockSourceIf.hpp"
#include "ps/PowerModelChannelIf.hpp"
#include <ostream>
#include <systemc>
#include <tlm>

/**
 * Description
 *
 * A configurable memory arranged in banks. Implements the following features:
 *
 *   - Configurable Volatile or Non-Volatile
 *   - Banked memory arrays
 *   - Control register that controls
 *     - Global bank power mode (active/low-power/off)
 *     - Automatic power mode selection
 *
 * The automatic power mode selection puts individual banks into a low-power
 * retention mode if they have not been accessed for a configurable number of
 * clock cycles.
 *
 */

namespace BankedMemory {

// Class to store register addresses for the control interface.
struct RegisterAddress {
  /* CONTROL
   * Bit-fields:
   *   0:       CONTROL_MODE, global control mode.
   *              0: MANUAL, set the power mode using the POWER_MODE register.
   *              1: AUTO, banks automatically go into/wake up from retention
   *                 mode.
   *   3..31:   UNUSED
   */
  static const unsigned CONTROL = 0x0;

  /* POWERMODE
   * Bit-fields (per bank n, up to 16 banks supported)
   *   n..n+1:    POWER_MODE
   *              0: OFF, memory accesses will result in error.
   *              1: RETENTION, memory accesses will cause bank to transition to
   *                 active mode (CONTROL_MODE=AUTO) or in an error
   *                 (CONTROL_MODE=MANUAL)
   *              2: ACTIVE, memory is active and reads take a single clock
   *                 cycle.
   *
   * e.g a value of 0x0002 means bank 0 is ACTIVE, all other banks are off
   *
   * Writing to this register when the CONTROL_MODE is AUTO results in an error.
   * This register can be read to find the current power mode of all banks.
   */
  static const unsigned POWERMODE = 0x4;
};

// Class to store bit masks etc. for the control interface.
struct BitMask {
  // Control register
  static const unsigned CONTROL_CONTROL_MODE_MASK = (1u << 0);
  static const unsigned CONTROL_CONTROL_MODE_MANUAL = (0u << 0);
  static const unsigned CONTROL_CONTROL_MODE_AUTO = (1u << 0);

  // Power mode register
  static const unsigned POWERMODE_BANK0_MASK = (0b11u << 0);
  static const unsigned POWERMODE_BANK0_OFF = (0u << 0);
  static const unsigned POWERMODE_BANK0_RETENTION = (1u << 0);
  static const unsigned POWERMODE_BANK0_ACTIVE = (2u << 0);
};

// Memory bank
class Bank : public sc_core::sc_module {
public:
  //! Event-port for logging and reporting dynamic power consumption
  PowerModelEventOutPort powerModelPort{"powerModelPort"};

  //! Clock
  sc_core::sc_port<ClockSourceConsumerIf> clk{"clk"};

  //! Reset
  sc_core::sc_in<bool> nReset{"nReset"};

  //! Control mode input:
  //!  - high: MANUAL
  //!  - low:  AUTO
  sc_core::sc_in<bool> SetControlMode{"SetControlMode"};

  // Signals to set power mode (only applies when ControlMode is high)
  sc_core::sc_in<bool> SetRetentionMode{"SetRetentionMode"};
  sc_core::sc_in<bool> SetActiveMode{"SetActiveMode"};
  sc_core::sc_in<bool> SetOffMode{"SetOffMode"};

  // Signals to report current power mode
  sc_core::sc_out<bool> IsRetentionMode{"IsRetentionMode"};
  sc_core::sc_out<bool> IsActiveMode{"IsActiveMode"};
  sc_core::sc_out<bool> IsOffMode{"IsOffMode"};

  /**
   * PowerMode
   *   Active: Memory is accessed with one cycle delay
   *   Retention: Memory can't be accessed
   *   OFF: Memory can't be accessed
   */
  enum class PowerMode { ACTIVE, RETENTION, OFF };

  /**
   * ControlMode
   *   MANUAL: the PowerMode is controlled via explicit transitions.
   *   AUTO: automatic control of PowerMode.
   */
  enum class ControlMode { MANUAL, AUTO };

  /**
   * AccessMode
   *   Indicate read/write accesses for transfers
   */
  enum class AccessMode { READ, WRITE };

  /**
   * @brief Bank constructor
   * @param capacity capacity in bytes.
   * @param autoRetentionDelayCycles number of clock cycles before automatically
   *        power down to retention mode (when SetControl==false).
   * @param activeToRetentionDelayCycles number of clock cycles it takes to
   *        switch from active to retention mode.
   * @param retentionToActiveDelayCycles number of clock cycles it takes to
   *        switch from retention to active mode.
   * @param isVolatile set to true if this memory should reset after power
   *        failures; false if this memory should retain contents through power
   *        failures.
   */
  Bank(sc_core::sc_module_name name, int capacity,
       int autoRetentionDelayCycles = 100,
       int activeToRetentionDelayCycles = 100,
       int retentionToActiveDelayCycles = 100, bool isVolatile = true);

  /**
   * @brief process SC_METHOD that handles control inputs and sets status
   * outputs.
   */
  void process();

  /**
   * @brief transfer perform a  read or write to this bank.
   * @param address address to read/write
   * @param buf pointer to write/read buffer
   * @param len size of transfer
   * @param delay
   */
  void transfer(unsigned address, uint8_t *buf, int len, AccessMode accessMode,
                sc_core::sc_time &delay);

  /**
   * @brief transfer_dbg transfer without side effects and delay.
   * @param address address to read/write
   * @param buf pointer to write/read buffer
   * @param len size of transfer
   */
  void transfer_dbg(unsigned address, uint8_t *buf, const int len,
                    const AccessMode accessMode);

  /**
   * @brief reset
   */
  void reset();

  /**
   * @brief set up SC_METHODs, event ID's etc.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const Bank &rhs);

  /* ------ Constants ------ */

  //! Delay (in clock cycles) before powering down to retention mode when in
  //! auto control mode.
  const unsigned AutoRetentionDelayCycles;

  //! How many clock cycles it takes to transition from active mode to retention
  //! mode
  const unsigned ActiveToRetentionDelayCycles;

  //! How many clock cycles it takes to transition from retention mode to active
  //! mode
  const unsigned RetentionToActiveDelayCycles;

  //! Volatility. If true, memory resets on every power failure. If false,
  //! memory contents are preserved through power failures.
  const bool m_isVolatile;

  /* ------ Variables ------ */
  //! Memory
  std::vector<uint8_t> m_data;

  //! Power mode
  PowerMode m_powerMode;
  bool m_setRetention{false};
  bool m_setActive{false};

  //! Signals that the power mode should change.
  sc_core::sc_event m_modeChangedEvent{"modeChangedEvent"};

  //! Power modelling events & states
  int m_activeStateId{-1};
  int m_retentionStateId{-1};
  int m_offStateId{-1};
  int m_writeEventId{-1};
  int m_readEventId{-1};
  int m_retentionToActiveEventId{-1};
  int m_activeToRetentionEventId{-1};
};

// Controller, implements bus control interface
class Controller : public BusTarget {
public:
  //! Control mode output:
  //!  - high: MANUAL
  //!  - low:  AUTO
  sc_core::sc_out<bool> SetControlMode;

  // Signals to set power mode for each bank (only applies when ControlMode is
  // high)
  std::vector<std::unique_ptr<sc_core::sc_out<bool>>> SetRetentionMode;
  std::vector<std::unique_ptr<sc_core::sc_out<bool>>> SetActiveMode;
  std::vector<std::unique_ptr<sc_core::sc_out<bool>>> SetOffMode;

  // Signals to get current power mode from each bank
  std::vector<std::unique_ptr<sc_core::sc_in<bool>>> IsRetentionMode;
  std::vector<std::unique_ptr<sc_core::sc_in<bool>>> IsActiveMode;
  std::vector<std::unique_ptr<sc_core::sc_in<bool>>> IsOffMode;

  /**
   * @brief Controller constructor.
   */
  Controller(sc_core::sc_module_name nm, unsigned startAddress, int nBanks);

  /**
   * @brief reset Reset to power-on defaults.
   */
  virtual void reset() override;

  /**
   * @brief set up methods/threads after module construction complete.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief process main process loop
   */
  void process();

  /**
   * @brief b_transport override transport to prepare status register when it is
   * needed.
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /**
   * @brief << debug printout.
   */
  friend std::ostream &operator<<(std::ostream &os, const Controller &rhs);

  /* ------ Constants ------ */
  const int m_nBanks;
};

class BankedMemory : public BusTarget {

public:
  // Constructor
  BankedMemory(sc_core::sc_module_name name, unsigned memoryStartAddress,
               unsigned controlStartAddress, int nBanks, int bankSize,
               bool isVolatile = true);

  /**
   * @brief reset memory and controller to power-on defaults.
   */
  virtual void reset() override;

  /**
   * @brief set up methods/threads after module construction complete.
   */
  virtual void end_of_elaboration() override;

  /**
   * @brief b_transport override to access banks
   */
  virtual void b_transport(tlm::tlm_generic_payload &trans,
                           sc_core::sc_time &delay) override;

  /**
   * @brief transport_dbg override to access banks
   */
  virtual unsigned transport_dbg(tlm::tlm_generic_payload &trans) override;

  /* ------ Submodules ------ */
  std::vector<std::unique_ptr<Bank>> m_banks;
  Controller m_ctrl;

  /* ------ Signals ------ */
  //! Control mode:
  //!  - high: MANUAL
  //!  - low:  AUTO
  sc_core::sc_signal<bool> SetControlMode{"SetControlMode"};

  // Signals to set power mode for each bank (only applies when ControlMode is
  // high)
  std::vector<std::unique_ptr<sc_core::sc_signal<bool>>> SetRetentionMode;
  std::vector<std::unique_ptr<sc_core::sc_signal<bool>>> SetActiveMode;
  std::vector<std::unique_ptr<sc_core::sc_signal<bool>>> SetOffMode;

  // Signals to get current power mode from each bank
  std::vector<std::unique_ptr<sc_core::sc_signal<bool>>> IsRetentionMode;
  std::vector<std::unique_ptr<sc_core::sc_signal<bool>>> IsActiveMode;
  std::vector<std::unique_ptr<sc_core::sc_signal<bool>>> IsOffMode;

  /* ------ Constants ------ */
  const int m_nBanks;
  const int m_bankSize;
  const bool m_isVolatile;
};

} // namespace BankedMemory
