/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <spdlog/spdlog.h>
#include <systemc>
#include "sd/Bme280.hpp"

using namespace sc_core;

Bme280::Bme280(const sc_module_name name)
    : SpiDevice(name, ChipSelectPolarity::ActiveLow) {
  // Initialise memory mapped control registers.
  m_regs.addRegister(0);
  m_regs.addRegister(ADDR_HUM_LSB, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_HUM_MSB, 0x80, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_TEMP_XLSB, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_TEMP_LSB, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_TEMP_MSB, 0x80, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_PRESS_XLSB, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_PRESS_LSB, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_PRESS_MSB, 0x80, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CONFIG, 0, RegisterFile::AccessMode::READ_WRITE,
                     /*mask=*/~(1u << 1));
  m_regs.addRegister(ADDR_CTRL_MEAS, 0, RegisterFile::AccessMode::READ_WRITE);
  m_regs.addRegister(ADDR_STATUS, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CTRL_HUM, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_RESET, 0, RegisterFile::AccessMode::READ_WRITE);
  m_regs.addRegister(ADDR_ID, 0x60, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_00, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_01, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_02, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_03, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_04, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_05, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_06, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_07, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_08, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_09, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_10, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_11, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_12, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_13, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_14, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_15, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_16, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_17, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_18, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_19, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_20, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_21, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_22, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_23, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_24, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_25, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_26, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_27, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_28, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_29, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_30, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_31, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_32, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_33, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_34, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_35, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_36, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_37, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_38, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_39, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_40, 0, RegisterFile::AccessMode::READ);
  m_regs.addRegister(ADDR_CALIB_41, 0, RegisterFile::AccessMode::READ);

  SC_METHOD(spiInterface);
  sensitive << m_transactionEvent;
  dont_initialize();
}

void Bme280::reset(void) {
  m_regs.reset();
  SpiDevice::reset();
}

void Bme280::spiInterface(void) {
  // First word after chip select is address, remaining are data
  const auto payload = readSlaveIn();
  spdlog::info("{:s}: @{:s} Received 0x{:08x}", this->name(),
               sc_time_stamp().to_string(), payload);

  if (chipSelect.read() == false) {  // Active
    switch (m_spiState) {
      case SpiState::Address:
        m_activeAddress = payload;
        m_spiState = SpiState::Data;
        break;
      case SpiState::Data:
        if (!(m_activeAddress & (1u << 7))) {  // Write access
          switch (m_activeAddress) {
            case ADDR_CONFIG:
              m_regs.write(m_activeAddress & 0x7F, payload);  // 7-bit address
              break;
            case ADDR_CTRL_MEAS:
              m_regs.write(m_activeAddress & 0x7F, payload);  // 7-bit address
              break;
            case ADDR_RESET:
              if (payload == 0xB6) {  // Magic number to trigger reset
                reset();
              } else {
                // otherwise, ignore
              }
              break;
            default:
              SC_REPORT_FATAL(
                  this->name(),
                  fmt::format("Invalid write access to address 0x{:08x}, which "
                              "is not a writeable register.",
                              m_activeAddress)
                      .c_str());
              break;
          }
          m_spiState = SpiState::Address;  // Next payload will be an address
        }
        break;
    }

    // Prepare response
    if ((m_activeAddress & (1u << 7)) && m_regs.contains(m_activeAddress)) {
      // Valid read-access
      spdlog::info("{:s}: setting response to regs[0x{:02x}] = 0x{:2x}",
                   this->name(), m_activeAddress, m_regs.read(m_activeAddress));
      writeSlaveOut(m_regs.read(m_activeAddress));
      ++m_activeAddress;  // Auto-increment address
    } else {
      writeSlaveOut(0);
    }

  } else if (chipSelect.read()) {  // Inactive
    m_spiState = SpiState::Address;
  }
}

// Override parent b_transport to check for compatible spi settings
