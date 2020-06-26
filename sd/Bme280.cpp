/*
 * Copyright (c) 2019-2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <spdlog/spdlog.h>
#include <systemc>
#include <tuple>
#include <vector>
#include "libs/strtk.hpp"
#include "sd/Bme280.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

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
  m_regs.addRegister(ADDR_CTRL_HUM, 0, RegisterFile::AccessMode::READ_WRITE);
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

  // Load boot current trace
  if (Config::get().contains("Bme280TraceFile")) {
    auto fn = Config::get().getString("Bme280TraceFile");
    Utility::assertFileExists(fn);
    std::ifstream file(fn);
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    strtk::token_grid grid(content, content.size(), ",");
    for (std::size_t i = 0; i < grid.row_count(); ++i) {
      m_inputTrace.push_back(InputTraceEntry(
          /*Temperature*/ grid.row(i).get<double>(1),
          /*Humidity*/ grid.row(i).get<double>(2),
          /*Pressure*/ grid.row(i).get<double>(3)));
    }
    m_inputTraceTimestep = sc_time(
        1000 * (grid.row(1).get<double>(0) - grid.row(0).get<double>(0)),
        SC_MS);
  } else {  // No boot trace specified, set constant
    m_inputTrace.push_back(InputTraceEntry(
        /*Temperature*/ 20.0,
        /*Humidity*/ 30.0,
        /*Pressure*/ 330.0));
    m_inputTraceTimestep = sc_time(1000.0, SC_MS);
  }
}

void Bme280::end_of_elaboration() {
  SC_METHOD(spiInterface);
  sensitive << m_transactionEvent << chipSelect.posedge_event();
  dont_initialize();

  SC_THREAD(measurementLoop);
}

void Bme280::reset(void) {
  m_regs.reset();
  SpiDevice::reset();
}

void Bme280::spiInterface(void) {
  // First word after chip select is address, remaining are data
  const unsigned READ_BIT = (1u << 7);
  const auto payload = readSlaveIn();
  if (enabled()) {  // Chip select active
    switch (m_spiState) {
      case SpiState::Address:
        m_activeAddress = payload | READ_BIT;
        m_isWriteAccess = !(payload & READ_BIT);
        m_spiState = SpiState::Data;
        break;
      case SpiState::Data:
        if (m_isWriteAccess) {  // Write access
          switch (m_activeAddress) {
            case ADDR_CONFIG:
              m_regs.write(m_activeAddress, payload);
              break;
            case ADDR_CTRL_HUM:
              m_regs.write(m_activeAddress, payload);
              break;
            case ADDR_CTRL_MEAS:
              m_regs.write(m_activeAddress, payload);
              m_modeUpdateEvent.notify(SC_ZERO_TIME);
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
    if ((!m_isWriteAccess) && m_regs.contains(m_activeAddress)) {
      // Valid read-access
      writeSlaveOut(m_regs.read(m_activeAddress));
      ++m_activeAddress;  // Auto-increment address
    } else {              // Write access, or invalid address
      writeSlaveOut(0);
    }
  } else if (!enabled()) {  // Chip select inactive
    m_spiState = SpiState::Address;
  }
}

Bme280::MeasurementState Bme280::nextMeasurementState() const {
  auto mode = m_regs.read(ADDR_CTRL_MEAS) & 0b11u;
  MeasurementState result = m_measurementState;
  switch (m_measurementState) {
    case MeasurementState::PowerOff:
      // Check VDD and VDDIO
      break;
    case MeasurementState::Sleep:
      if (mode == 1 || mode == 2) {
        result = MeasurementState::Forced;
      } else if (mode == 3) {
        result = MeasurementState::Normal;
      }
      break;
    case MeasurementState::Normal:
      if (mode == 0) {
        result = MeasurementState::Sleep;
      } else if (mode == 1 || mode == 2) {
        result = MeasurementState::Forced;
      }
      break;
    case MeasurementState::Forced:
      result = MeasurementState::Sleep;
      break;
  }
  return result;
}

void Bme280::measurementLoop() {
  wait(m_modeUpdateEvent);

  while (1) {
    // State machine model
    m_measurementState = nextMeasurementState();

    // Clear mode bits after forced mode
    if (m_measurementState == MeasurementState::Forced) {
      m_regs.clearBitMask(ADDR_CTRL_MEAS, 0b11u, /*forced=*/true);
    }

    // Take a series of measurements
    if (m_measurementState == MeasurementState::Normal ||
        m_measurementState == MeasurementState::Forced) {
      unsigned result = 0;
      wait(sc_time(1, SC_MS));  // Constant part of t_measure (datasheet)

      // Get current sample (loops through input trace)
      InputTraceEntry input =
          m_inputTrace[static_cast<unsigned>(sc_time_stamp() /
                                             m_inputTraceTimestep) %
                       m_inputTrace.size()

      ];

      // Lambda for calculating oversampling factor
      auto nSamples = [](unsigned samplingFactor) -> unsigned {
        if (samplingFactor == 0) {
          return 0u;
        } else if (samplingFactor < 5) {
          return 1u << (samplingFactor - 1);
        } else {
          return 16u;
        }
      };

      // Lambda for running iir filter
      unsigned filter_coeff = nSamples((m_regs.read(ADDR_CONFIG) & 0x1c) >> 2);
      auto iir = [filter_coeff](unsigned oldval, unsigned newval) -> unsigned {
        if (filter_coeff == 0) {
          return newval;
        } else {
          return (oldval * (filter_coeff - 1) + newval) / filter_coeff;
        }
      };

      m_regs.setBit(ADDR_STATUS, 3, true);  // Indicate measurement
      spdlog::info("{:s}: @{} starting measurement", this->name(),
                   sc_time_stamp().to_string());

      // Measure temperature
      auto osrs_t = (m_regs.read(ADDR_CTRL_MEAS) & 0xe0) >> 5;
      if (osrs_t == 0) {
        result = 0x8000;
      } else {
        EventLog::getInstance().reportState(this->name(),
                                            "measure_temperature");
        for (int i = 0; i < nSamples(osrs_t); ++i) {
          wait(sc_time(2, SC_MS));  // Sampling time
          result += static_cast<unsigned>((input.temperature - TEMP_OFFSET) /
                                          TEMP_SCALE);
        }
        result /= nSamples(osrs_t);  // Average of oversampling
        auto oldval = m_regs.read(ADDR_TEMP_XLSB) |
                      (m_regs.read(ADDR_TEMP_LSB) << 4) |
                      (m_regs.read(ADDR_TEMP_MSB) << 12);
        result = iir(oldval, result);
        spdlog::info("{:s}: @{} Temperature measurement 0x{:04x} ({:.3f} C)",
                     this->name(), sc_time_stamp().to_string(), result,
                     (result * TEMP_SCALE) + TEMP_OFFSET);
      }
      m_regs.write(ADDR_TEMP_XLSB, result & 0x0f, true);
      m_regs.write(ADDR_TEMP_LSB, (result & 0x0ff0) >> 4, true);
      m_regs.write(ADDR_TEMP_MSB, (result & 0xff000) >> 12, true);

      // Measure pressure
      auto osrs_p = (m_regs.read(ADDR_CTRL_MEAS) & 0x1c) >> 2;
      if (osrs_p == 0) {
        result = 0x8000;
      } else {
        result = 0;
        EventLog::getInstance().reportState(this->name(), "measure_pressure");
        for (int i = 0; i < nSamples(osrs_p); ++i) {
          wait(sc_time(2, SC_MS));  // Sampling time
          result += static_cast<unsigned>((input.pressure - PRESS_OFFSET) /
                                          PRESS_SCALE);
        }
        wait(sc_time(0.5,
                     SC_MS));        // Constant part of pressure sampling time
        result /= nSamples(osrs_p);  // Average of oversampling
        auto oldval = m_regs.read(ADDR_PRESS_XLSB) |
                      (m_regs.read(ADDR_PRESS_LSB) << 4) |
                      (m_regs.read(ADDR_PRESS_MSB) << 12);
        result = iir(oldval, result);
        spdlog::info("{:s}: @{} Pressure measurement 0x{:04x} ({:.3f} hPa)",
                     this->name(), sc_time_stamp().to_string(), result,
                     (result * PRESS_SCALE) + PRESS_OFFSET);
      }
      m_regs.write(ADDR_PRESS_XLSB, result & 0x0f, true);
      m_regs.write(ADDR_PRESS_LSB, (result & 0x0ff0) >> 4, true);
      m_regs.write(ADDR_PRESS_MSB, (result & 0xff000) >> 12, true);

      // Measure humidity
      auto osrs_h = m_regs.read(ADDR_CTRL_HUM);
      if (osrs_h == 0) {
        result = 0x8000;
      } else {
        result = 0;
        EventLog::getInstance().reportState(this->name(), "measure_humidity");
        for (int i = 0; i < nSamples(osrs_h); ++i) {
          wait(sc_time(2, SC_MS));  // Sampling time
          result +=
              static_cast<unsigned>((input.humidity - HUM_OFFSET) / HUM_SCALE);
        }
        result /= nSamples(osrs_h);
        spdlog::info("{:s}: @{} Humidity measurement 0x{:04x} ({:.3f} %RH)",
                     this->name(), sc_time_stamp().to_string(), result,
                     (result * HUM_SCALE) + HUM_OFFSET);
        // no iir for humidity
        wait(sc_time(0.5,
                     SC_MS));  // Constant part of humidity sampling time
      }

      // Store result
      m_regs.write(ADDR_HUM_LSB, result & 0xff, true);
      m_regs.write(ADDR_HUM_MSB, (result & 0xff00) >> 8, true);
      m_regs.clearBit(ADDR_STATUS, 3, true);

      if (m_measurementState == MeasurementState::Normal) {
        // Standby wait time
        EventLog::getInstance().reportState(this->name(), "standby");
        const int STANDBY_DELAY_US[] = {500,    62500,   125000, 250000,
                                        500000, 1000000, 10000,  20000};
        auto delay = sc_time(
            STANDBY_DELAY_US[(m_regs.read(ADDR_CONFIG) & 0xe0) >> 5], SC_US);
        spdlog::info("{:s}: @{:s} waiting in standby for {:s}", this->name(),
                     sc_time_stamp().to_string(), delay.to_string());
        wait(delay);
      } else if (m_measurementState == MeasurementState::Sleep) {
        EventLog::getInstance().reportState(this->name(), "sleep");
      } else if (m_measurementState == MeasurementState::PowerOff) {
        EventLog::getInstance().reportState(this->name(), "off");
      }
    } else {  // Inactive
      wait(m_modeUpdateEvent);
    }
  }
}

// Override parent b_transport to check for compatible spi settings
