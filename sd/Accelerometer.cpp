/*
 * Copyright (c) 2020, University of Southampton and Contributors.
 * All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <spdlog/spdlog.h>
#include <systemc>
#include <tuple>
#include <vector>
#include "libs/strtk.hpp"
#include "sd/Accelerometer.hpp"
#include "utilities/Config.hpp"
#include "utilities/Utilities.hpp"

using namespace sc_core;

Accelerometer::Accelerometer(const sc_module_name name)
    : SpiDevice(name, ChipSelectPolarity::ActiveLow) {
  // Initialise memory mapped control registers.
  m_regs.addRegister(RegisterAddress::CTRL);
  m_regs.addRegister(RegisterAddress::CTRL_FIFO);
  m_regs.addRegister(RegisterAddress::CTRL_FS);
  m_regs.addRegister(RegisterAddress::STATUS,
                     /*resetValue=*/BitMasks::STATUS_BUSY);
  m_regs.addRegister(RegisterAddress::DATA);

  // Load sensor input trace
  if (Config::get().contains("AccelerometerTraceFile")) {
    auto fn = Config::get().getString("AccelerometerTraceFile");
    Utility::assertFileExists(fn);
    std::ifstream file(fn);
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    strtk::token_grid grid(content, content.size(), ",");
    for (std::size_t i = 0; i < grid.row_count(); ++i) {
      m_inputTrace.push_back({/*acc_x*/ grid.row(i).get<double>(1),
                              /*acc_y*/ grid.row(i).get<double>(2),
                              /*acc_z*/ grid.row(i).get<double>(3)});
    }
    m_inputTraceTimestep = sc_time(
        1000 * (grid.row(1).get<double>(0) - grid.row(0).get<double>(0)),
        SC_MS);
  } else {  // No boot trace specified, set constant
    m_inputTrace.push_back(InputTraceEntry{/*acc_x*/ 0.0,
                                           /*acc_y*/ 0.0,
                                           /*acc_z*/ 9.81});
    m_inputTraceTimestep = sc_time(1000.0, SC_MS);
  }
}

void Accelerometer::end_of_elaboration() {
  SC_METHOD(spiInterface);
  sensitive << m_transactionEvent << chipSelect.posedge_event();
  dont_initialize();

  SC_THREAD(measurementLoop);
}

void Accelerometer::reset(void) {
  m_regs.reset();
  SpiDevice::reset();
  m_fifo.reset();
}

void Accelerometer::spiInterface(void) {
  // First word after chip select is address, remaining are data
  const unsigned READ_BIT = (1u << 7);
  const auto payload = readSlaveIn();
  if (enabled()) {  // Chip select active
    switch (m_spiState.mode) {
      case SpiState::Mode::Address:  // First byte sets up address
        m_spiState.address = payload | READ_BIT;
        m_spiState.mode = (payload & READ_BIT) ? SpiState::Mode::DataRead
                                               : SpiState::Mode::DataWrite;
        break;
      case SpiState::Mode::DataRead:
        break;
      case SpiState::Mode::DataWrite:  // Commands
        if (!m_regs.contains(m_spiState.address)) {
          SC_REPORT_FATAL(this->name(),
                          fmt::format("Invalid SPI write address 0x{:08x}",
                                      m_spiState.address)
                              .c_str());
        }
        m_regs.write(m_spiState.address, payload);

        switch (m_spiState.address) {
          case RegisterAddress::CTRL:
            if (payload & BitMasks::CTRL_SW_RESET) {
              reset();
            } else {
              m_modeUpdateEvent.notify(SC_ZERO_TIME);
            }
            break;
          default:
            break;
        }
        m_spiState.mode = SpiState::Mode::Address;
        break;
    }

    // Prepare response to next transaction
    if (m_spiState.mode != SpiState::Mode::DataWrite) {
      if (!m_regs.contains(m_spiState.address)) {
        SC_REPORT_FATAL(
            this->name(),
            fmt::format("Invalid SPI read address 0x{:08x}", m_spiState.address)
                .c_str());
      }
      if (m_spiState.address == RegisterAddress::DATA) {
        // Special case -- fifo data
        writeSlaveOut(m_fifo.takeByte());
      } else {
        writeSlaveOut(m_regs.read(m_spiState.address));
      }
    } else {
      writeSlaveOut(0);
    }

  } else if (!enabled()) {  // Chip select inactive
    m_spiState.mode = SpiState::Mode::Address;
  }
}

Accelerometer::MeasurementState Accelerometer::nextMeasurementState() const {
  auto setting = static_cast<MeasurementState>(
      m_regs.read(RegisterAddress::CTRL) & BitMasks::CTRL_MODE);
  // TODO: can't go from Off to measure without going through standby
  // TODO: Delays between states
  return setting;
}

void Accelerometer::measurementLoop() {
  wait(m_modeUpdateEvent);

  while (1) {
    // State machine model
    m_measurementState = nextMeasurementState();

    // Go back to standby after single measurement
    if (m_measurementState == MeasurementState::SingleMeasurement) {
      m_regs.clearBitMask(RegisterAddress::CTRL, BitMasks::CTRL_MODE);
    }

    // Take a series of measurements
    if (m_measurementState == MeasurementState::ContinuousMeasurement ||
        m_measurementState == MeasurementState::SingleMeasurement) {
      // Indicate busy
      m_regs.setBitMask(RegisterAddress::STATUS, BitMasks::STATUS_BUSY);

      // Sampling time: 10KHz/divider
      wait(sc_time(0.1, SC_MS) * (m_regs.read(RegisterAddress::CTRL_FS) + 1));

      // Get current sample (wraps around input trace)
      InputTraceEntry input =
          m_inputTrace[static_cast<unsigned>(sc_time_stamp() /
                                             m_inputTraceTimestep) %
                       m_inputTrace.size()];

      // Lambda to convert trace values into bits
      auto sampleTrace = [](const double val) -> uint8_t {
        return static_cast<uint8_t>(ACC_OFFSET + ACC_SCALE * val);
      };

      // Sample axes
      const auto ctrl = m_regs.read(RegisterAddress::CTRL);
      FifoFrame frame;
      if (ctrl & BitMasks::CTRL_X_EN) {
        frame.header |= FifoFrame::HeaderFields::X_AXIS_ENABLE;
        frame.data.push_front(sampleTrace(input.acc_x));
      }
      if (ctrl & BitMasks::CTRL_Y_EN) {
        frame.header |= FifoFrame::HeaderFields::Y_AXIS_ENABLE;
        frame.data.push_front(sampleTrace(input.acc_y));
      }
      if (ctrl & BitMasks::CTRL_Z_EN) {
        frame.header |= FifoFrame::HeaderFields::Z_AXIS_ENABLE;
        frame.data.push_front(sampleTrace(input.acc_z));
      }

      // Store result
      m_fifo.push_front(frame);
      // TODO send pulse on IRQ when fifo overflows

    } else {  // Inactive
      wait(m_modeUpdateEvent);
    }
  }
}

// Override parent b_transport to check for compatible spi settings
